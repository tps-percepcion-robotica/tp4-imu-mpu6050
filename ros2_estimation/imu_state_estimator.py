#!/usr/bin/env python3
"""Parte 1.1: actitud, velocidad y posicion a partir de los datos crudos del MPU-6050.

Suscribe a sensor_msgs/Imu (por defecto /imu/data_raw), calibra con el sensor quieto,
estima la actitud con un filtro complementario (giroscopio + acelerometro),
integra la aceleracion para obtener velocidad y posicion, y publica:
  - TF  odom -> imu_link                 (actitud + posicion, para RViz2)
  - ~/odom             nav_msgs/Odometry (pose + velocidad)
  - ~/velocity_marker  Marker (flecha con el vector velocidad)
  - ~/reset            servicio std_srvs/Empty: pone velocidad y posicion en cero

Ademas abre una ventana con graficos en vivo (actitud, aceleracion sin gravedad,
velocidad y posicion). Con la tecla R sobre la ventana se reinician velocidad y posicion.

Uso:  python3 imu_fusion.py
      python3 imu_fusion.py --ros-args -p plot:=false     (sin graficos)
      python3 imu_fusion.py --ros-args -p zupt:=true      (velocidad = 0 al detectar reposo)
"""
import math
import threading
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
        super().__init__('imu_fusion')
        self.topic = self.declare_parameter('topic', 'imu/data_raw').value
        self.child_frame = self.declare_parameter('child_frame', 'imu_link').value
        self.kp = self.declare_parameter('kp', 2.0).value            # peso del acelerometro
        self.n_calib = self.declare_parameter('calib_samples', 300).value
        # True = usar la orientacion que viene en el mensaje (para la parte del DMP)
        self.use_msg_q = self.declare_parameter('use_msg_orientation', False).value

        # ZUPT (zero velocity update): si detecta reposo, fuerza velocidad = 0
        self.zupt = self.declare_parameter('zupt', False).value
        self.zupt_gyro = self.declare_parameter('zupt_gyro', 0.05).value   # rad/s
        self.zupt_acc = self.declare_parameter('zupt_acc', 0.3).value      # m/s2
        self.still_count = 0

        self.plot = self.declare_parameter('plot', True).value
        self.plot_window = self.declare_parameter('plot_window_s', 20.0).value

        # Historial para los graficos: (t, roll, pitch, yaw, ax, ay, az, vx, vy, vz, px, py, pz)
        self.hist = deque(maxlen=int(self.plot_window * 100))
        self.lock = threading.Lock()
        self.t_rel = 0.0

        self.count = 0
        self.sum_acc = np.zeros(3)
        self.sum_gyr = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.g = 9.80665
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.t_prev = None

        self.tf_br = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '~/odom', 10)
        self.pub_marker = self.create_publisher(Marker, '~/velocity_marker', 10)
        self.create_service(Empty, '~/reset', self.on_reset)
        self.create_subscription(Imu, self.topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info('Calibrando: dejar el sensor QUIETO unos segundos...')

    def reset(self):
        self.vel[:] = 0.0
        self.pos[:] = 0.0
        self.get_logger().info('Velocidad y posicion reiniciadas')

    def on_reset(self, request, response):
        self.reset()
        return response

    def on_imu(self, msg):
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                        msg.angular_velocity.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ---- Calibracion inicial (sensor quieto) ----
        if self.count < self.n_calib:
            self.sum_acc += acc
            self.sum_gyr += gyr
            self.count += 1
            self.t_prev = t
            if self.count == self.n_calib:
                mean_acc = self.sum_acc / self.n_calib
                self.gyro_bias = self.sum_gyr / self.n_calib
                self.g = float(np.linalg.norm(mean_acc))
                self.q = q_from_accel(mean_acc)
                self.get_logger().info(
                    f'Listo. Bias gyro [rad/s]: {np.round(self.gyro_bias, 4)}, '
                    f'|g| medido: {self.g:.3f} m/s2')
            return

        dt = t - self.t_prev
        self.t_prev = t
        if not 0.0 < dt < 0.1:      # timestamp invalido o salto: usar periodo nominal
            dt = 0.01

        # ---- Actitud ----
        if self.use_msg_q:
            o = msg.orientation
            q = np.array([o.w, o.x, o.y, o.z])
            n = np.linalg.norm(q)
            if n < 1e-6:
                return
            self.q = q / n
        else:
            w = gyr - self.gyro_bias
            a_norm = np.linalg.norm(acc)
            # Corregir con el acelerometro solo si mide ~1 g (sin aceleracion propia)
            if abs(a_norm - self.g) < 0.1 * self.g:
                up_est = q_to_rot(self.q)[2, :]        # "arriba" estimado, en ejes del sensor
                err = np.cross(acc / a_norm, up_est)
                w = w + self.kp * err
            dq = 0.5 * q_mult(self.q, np.array([0.0, w[0], w[1], w[2]]))
            self.q = self.q + dq * dt
            self.q /= np.linalg.norm(self.q)

        # ---- Velocidad y posicion: rotar al mundo, quitar gravedad, integrar ----
        acc_world = q_to_rot(self.q) @ acc - np.array([0.0, 0.0, self.g])
        self.vel += acc_world * dt
        if self.zupt:
            still = (np.linalg.norm(gyr - self.gyro_bias) < self.zupt_gyro
                     and abs(np.linalg.norm(acc) - self.g) < self.zupt_acc)
            self.still_count = self.still_count + 1 if still else 0
            if self.still_count >= 20:       # ~0.2 s seguidos en reposo
                self.vel[:] = 0.0
        self.pos += self.vel * dt

        self.t_rel += dt
        with self.lock:
            self.hist.append((self.t_rel, *np.degrees(q_to_euler(self.q)),
                              *acc_world, *self.vel, *self.pos))

        self.publish(acc_world)

    def publish(self, acc_world):
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
        # Velocidad expresada en el marco odom (mundo)
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
        mk.scale.x = 0.02   # diametro del cuerpo de la flecha
        mk.scale.y = 0.04   # diametro de la punta
        mk.color.r = 1.0
        mk.color.g = 0.6
        mk.color.a = 1.0
        self.pub_marker.publish(mk)


def run_plot(node):
    """Graficos en vivo. Bloquea hasta que se cierra la ventana."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    panels = [
        ('Actitud [grados]', ('roll', 'pitch', 'yaw')),
        ('Aceleracion en el mundo, sin gravedad [m/s2]', ('x', 'y', 'z')),
        ('Velocidad [m/s]', ('x', 'y', 'z')),
        ('Posicion [m]', ('x', 'y', 'z')),
    ]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
    metodo = 'orientacion del DMP' if node.use_msg_q else 'filtro complementario propio'
    if node.zupt:
        metodo += ' + ZUPT'
    fig.suptitle(f'{node.get_name()}: {metodo}', fontsize=11)
    lines = []
    for ax, (title, labels) in zip(axes, panels):
        ax.set_title(title, fontsize=10)
        ax.grid(True)
        for label in labels:
            lines.append(ax.plot([], [], label=label)[0])
        ax.legend(loc='upper left', ncol=3, fontsize=8)
    axes[-1].set_xlabel('Tiempo [s]     (tecla R: reiniciar velocidad y posicion)')

    def on_key(event):
        if event.key in ('r', 'R'):
            node.reset()

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        with node.lock:
            data = np.array(node.hist)
        if len(data) < 2:
            return lines
        t = data[:, 0]
        for i, line in enumerate(lines):
            line.set_data(t, data[:, i + 1])
        for ax in axes:
            ax.relim()
            ax.autoscale_view()
        axes[0].set_xlim(t[0], max(t[-1], t[0] + 1.0))
        return lines

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    fig.tight_layout()
    plt.show()


def spin_node(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node = ImuFusion()
    try:
        if node.plot:
            threading.Thread(target=spin_node, args=(node,), daemon=True).start()
            run_plot(node)          # la ventana corre en el hilo principal
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()