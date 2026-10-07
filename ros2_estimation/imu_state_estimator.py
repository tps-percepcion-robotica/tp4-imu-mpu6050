#!/usr/bin/env python3
"""Parte 1.1 y 1.3: Comparación de actitud, velocidad y posición (DMP vs Comp vs Madgwick).

Suscribe a sensor_msgs/Imu (/imu/data_raw), calibra con el sensor quieto,
y estima el estado simultáneamente con 3 métodos:
  0: DMP (Hardware del MPU-6050)
  1: Filtro Complementario (Mahony proporcional)
  2: Filtro de Madgwick

Publica:
  - 3 TFs (odom -> imu_link_dmp / _comp / _madg) para visualización simultánea en RViz2.
  - Abre una ventana comparando Roll, Pitch, Yaw y Posición Z.
"""
import fcntl
import math
import os
import threading
from datetime import datetime
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def q_mult(a, b):
    """Producto de cuaterniones [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q):
    """Conjugado (inversa de un cuaternion unitario)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_to_rot(q):
    """Matriz de rotacion cuerpo -> mundo."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_from_accel(acc):
    """Roll y pitch iniciales a partir de la gravedad medida (yaw = 0)."""
    ax, ay, az = acc
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


def q_to_euler(q):
    """Roll, pitch, yaw [rad] a partir del cuaternion [w, x, y, z]."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class ImuFusion(Node):
    def __init__(self):
        super().__init__('imu_estimator')
        self.topic = self.declare_parameter('topic', 'imu/data_raw').value
        self.child_frame = self.declare_parameter('child_frame', 'imu_link').value
        
        # Selección de algoritmo
        self.use_msg_q = self.declare_parameter('use_msg_orientation', False).value
        self.filter_type = self.declare_parameter('filter_type', 'complementary').value
        self.kp = self.declare_parameter('kp', 2.0).value
        self.beta = self.declare_parameter('beta', 0.1).value
        self.n_calib = self.declare_parameter('calib_samples', 300).value
        # CSV donde se agrega una fila por cada calibracion ('' para desactivar)
        self.calib_log = self.declare_parameter('calib_log', 'calibraciones.csv').value
        self.t_calib0 = None

        # ZUPT (zero velocity update): si detecta reposo, fuerza velocidad = 0
        self.zupt = self.declare_parameter('zupt', False).value
        self.zupt_gyro = self.declare_parameter('zupt_gyro', 0.05).value   # rad/s
        self.zupt_acc = self.declare_parameter('zupt_acc', 0.3).value      # m/s2
        self.still_count = 0

        self.plot = self.declare_parameter('plot', True).value
        self.plot_window = self.declare_parameter('plot_window_s', 20.0).value

        # Historial
        self.hist = deque(maxlen=int(self.plot_window * 100))
        self.lock = threading.Lock()
        self.t_rel = 0.0

        self.count = 0
        self.sum_acc = np.zeros(3)
        self.sum_gyr = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.g = 9.80665

        # Estado
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_off = np.array([1.0, 0.0, 0.0, 0.0])   # alinea el DMP con el cero comun
        self.q_dmp = None                             # ultimo cuaternion crudo del DMP
        self.recalib = False                          # pedido de tecla R / servicio ~/reset
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.t_prev = None

        self.tf_br = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '~/odom', 10)
        self.pub_marker = self.create_publisher(Marker, '~/velocity_marker', 10)
        self.create_service(Empty, '~/reset', self.on_reset)
        self.create_subscription(Imu, self.topic, self.on_imu, qos_profile_sensor_data)
        
        alg_name = "DMP" if self.use_msg_q else self.filter_type.upper()
        self.get_logger().info(f'Iniciando [{alg_name}]. Calibrando sensor...')

    def reset(self):
        # Se aplica en on_imu para no tocar el estado desde otro hilo
        self.recalib = True
        self.get_logger().info('Reinicio pedido: dejar el sensor QUIETO ~2 s para recalibrar')

    def log_calibration(self, mean_acc, span):
        """Agrega una fila al CSV de calibraciones (lo leen los tres nodos a la vez)."""
        if not self.calib_log:
            return
        alg = 'DMP' if self.use_msg_q else self.filter_type
        # Mensajes recibidos por segundo: ~200 Hz (tasa del DMP); menos = se pierden mensajes
        rate = self.n_calib / span if span > 0 else 0.0
        row = [datetime.now().isoformat(timespec='seconds'), self.get_name(), alg,
               self.n_calib, f'{span:.2f}', f'{rate:.0f}',
               *[f'{b:.5f}' for b in self.gyro_bias],
               *[f'{a:.4f}' for a in mean_acc], f'{self.g:.4f}']
        header = ['fecha', 'nodo', 'metodo', 'muestras', 'duracion_s', 'frecuencia_hz',
                  'bias_gx_rad_s', 'bias_gy_rad_s', 'bias_gz_rad_s',
                  'acc_x_m_s2', 'acc_y_m_s2', 'acc_z_m_s2', 'g_medido_m_s2']
        try:
            with open(self.calib_log, 'a') as f:
                fcntl.flock(f, fcntl.LOCK_EX)   # evita que dos nodos escriban la misma linea
                if os.fstat(f.fileno()).st_size == 0:
                    f.write(','.join(header) + '\n')
                f.write(','.join(map(str, row)) + '\n')
        except OSError as e:
            self.get_logger().warn(f'No se pudo escribir {self.calib_log}: {e}')

    def on_reset(self, request, response):
        self.reset()
        return response

    def on_imu(self, msg):
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q_msg = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if np.linalg.norm(q_msg) > 1e-6:
            self.q_dmp = q_msg / np.linalg.norm(q_msg)

        if self.recalib:
            self.recalib = False
            self.count = 0
            self.sum_acc[:] = 0.0
            self.sum_gyr[:] = 0.0
            self.vel[:] = 0.0
            self.pos[:] = 0.0
            self.still_count = 0

        # ---- Calibración inicial (y recalibración con la tecla R) ----
        if self.count < self.n_calib:
            if self.count == 0:
                self.t_calib0 = t
            self.sum_acc += acc
            self.sum_gyr += gyr
            self.count += 1
            self.t_prev = t
            if self.count == self.n_calib:
                mean_acc = self.sum_acc / self.n_calib
                self.gyro_bias = self.sum_gyr / self.n_calib
                self.g = float(np.linalg.norm(mean_acc))
                self.q = q_from_accel(mean_acc)
                # Cero comun: los tres metodos arrancan con roll/pitch del acelerometro
                # y yaw = 0. El DMP tiene su propia referencia, asi que se guarda la
                # rotacion que lleva su orientacion actual a ese mismo cero.
                if self.use_msg_q and self.q_dmp is not None:
                    self.q_off = q_mult(self.q, q_conj(self.q_dmp))
                self.get_logger().info(
                    f'Listo. Bias gyro [rad/s]: {np.round(self.gyro_bias, 4)}, '
                    f'|g| medido: {self.g:.3f} m/s2')
                self.log_calibration(mean_acc, t - self.t_calib0)
            return

        dt = t - self.t_prev
        if dt <= 0.0:               # mensaje duplicado o desordenado: no integrarlo dos veces
            return
        self.t_prev = t
        if dt > 0.1:                # salto (se perdieron mensajes): usar periodo nominal
            dt = 0.01

        a_norm = np.linalg.norm(acc)
        w = gyr - self.gyro_bias

        # ---- Selección de Actitud ----
        if self.use_msg_q:
            # Hardware DMP
            if self.q_dmp is not None:
                self.q = q_mult(self.q_off, self.q_dmp)

        elif self.filter_type == 'madgwick':
            # Filtro Madgwick
            s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0
            qw, qx, qy, qz = self.q
            if a_norm > 0:
                ax, ay, az = acc / a_norm
                s0 = 4*qw*qy**2 + 2*qy*ax + 4*qw*qx**2 - 2*qx*ay
                s1 = 4*qx*qz**2 - 2*qz*ax + 4*qw**2*qx - 2*qw*ay - 4*qx + 8*qx**3 + 8*qx*qy**2 + 4*qx*az
                s2 = 4*qw**2*qy + 2*qw*ax + 4*qy*qz**2 - 2*qz*ay - 4*qy + 8*qy*qx**2 + 8*qy**3 + 4*qy*az
                s3 = 4*qx**2*qz - 2*qx*ax + 4*qy**2*qz - 2*qy*ay
                s_norm = math.sqrt(s0**2 + s1**2 + s2**2 + s3**2)
                if s_norm > 0:
                    s0 /= s_norm; s1 /= s_norm; s2 /= s_norm; s3 /= s_norm

            qDot = np.array([
                0.5 * (-qx*w[0] - qy*w[1] - qz*w[2]) - self.beta * s0,
                0.5 * ( qw*w[0] + qy*w[2] - qz*w[1]) - self.beta * s1,
                0.5 * ( qw*w[1] - qx*w[2] + qz*w[0]) - self.beta * s2,
                0.5 * ( qw*w[2] + qx*w[1] - qy*w[0]) - self.beta * s3
            ])
            self.q = self.q + qDot * dt
            self.q /= np.linalg.norm(self.q)

        else:
            # Filtro Complementario (Default)
            if abs(a_norm - self.g) < 0.1 * self.g:
                up_est = q_to_rot(self.q)[2, :]
                err = np.cross(acc / a_norm, up_est)
                w = w + self.kp * err
            dq = 0.5 * q_mult(self.q, np.array([0.0, w[0], w[1], w[2]]))
            self.q = self.q + dq * dt
            self.q /= np.linalg.norm(self.q)

        # ---- Cinemática ----
        acc_world = q_to_rot(self.q) @ acc - np.array([0.0, 0.0, self.g])
        self.vel += acc_world * dt
        
        if self.zupt:
            still = (np.linalg.norm(w) < self.zupt_gyro and abs(a_norm - self.g) < self.zupt_acc)
            self.still_count = self.still_count + 1 if still else 0
            if self.still_count >= 20:
                self.vel[:] = 0.0
        
        self.pos += self.vel * dt
        self.t_rel += dt

        with self.lock:
            self.hist.append((self.t_rel, *np.degrees(q_to_euler(self.q)), *acc_world, *self.vel, *self.pos))

        self.publish_tfs()

    def publish_tfs(self):
        now = self.get_clock().now().to_msg()
        w, x, y, z = self.q

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = float(self.pos[0])
        tf.transform.translation.y = float(self.pos[1])
        tf.transform.translation.z = float(self.pos[2])
        tf.transform.rotation.w = float(w)
        tf.transform.rotation.x = float(x)
        tf.transform.rotation.y = float(y)
        tf.transform.rotation.z = float(z)
        self.tf_br.sendTransform(tf)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = self.child_frame
        od.pose.pose.position.x = float(self.pos[0])
        od.pose.pose.position.y = float(self.pos[1])
        od.pose.pose.position.z = float(self.pos[2])
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x = float(self.vel[0])
        od.twist.twist.linear.y = float(self.vel[1])
        od.twist.twist.linear.z = float(self.vel[2])
        self.pub_odom.publish(od)

        mk = Marker()
        mk.header.stamp = now
        mk.header.frame_id = 'odom'
        mk.ns = self.get_name()
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        end = self.pos + self.vel
        mk.points = [Point(x=float(self.pos[0]), y=float(self.pos[1]), z=float(self.pos[2])),
                     Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
        mk.scale.x = 0.02
        mk.scale.y = 0.04
        mk.color.r = 0.0 if self.filter_type == 'madgwick' else 1.0
        mk.color.g = 1.0 if not self.use_msg_q else 0.0
        mk.color.b = 1.0 if self.filter_type == 'madgwick' else 0.0
        mk.color.a = 1.0
        self.pub_marker.publish(mk)

def run_plot(node):
    """Graficos en vivo. Bloquea hasta que se cierra la ventana."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    panels = [
        ('Actitud [grados]', ('roll', 'pitch', 'yaw')),
        ('Aceleracion en el mundo [m/s2]', ('x', 'y', 'z')),
        ('Velocidad [m/s]', ('x', 'y', 'z')),
        ('Posicion [m]', ('x', 'y', 'z')),
    ]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
    alg = "DMP" if node.use_msg_q else node.filter_type.capitalize()
    fig.suptitle(f'{node.get_name()}: {alg} {"+ ZUPT" if node.zupt else ""}', fontsize=11)
    
    lines = []
    for ax, (title, labels) in zip(axes, panels):
        ax.set_title(title, fontsize=10)
        ax.grid(True)
        for label in labels:
            lines.append(ax.plot([], [], label=label)[0])
        ax.legend(loc='upper left', ncol=3, fontsize=8)
    axes[-1].set_xlabel('Tiempo [s]     (tecla R: reiniciar)')

    def on_key(event):
        if event.key in ('r', 'R'):
            node.reset()
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        with node.lock:
            data = np.array(node.hist)
        if len(data) < 2: return lines
        t = data[:, 0]
        for i, line in enumerate(lines):
            line.set_data(t, data[:, i + 1])
        for ax in axes:
            ax.relim()
            ax.autoscale_view()
        axes[0].set_xlim(t[0], max(t[-1], t[0] + 1.0))
        return lines

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False) # noqa: F841
    fig.tight_layout()
    plt.show()

def spin_node(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'El nodo se detuvo: {e!r}')


def main():
    rclpy.init()
    node = ImuFusion()
    try:
        if node.plot:
            threading.Thread(target=spin_node, args=(node,), daemon=True).start()
            run_plot(node)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()