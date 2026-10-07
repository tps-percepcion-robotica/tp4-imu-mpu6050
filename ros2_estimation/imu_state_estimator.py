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
import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


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
        super().__init__('imu_fusion_compare')
        self.topic = self.declare_parameter('topic', 'imu/data_raw').value
        self.kp_comp = self.declare_parameter('kp_comp', 2.0).value  # peso del acelerometro
        self.beta_madg = self.declare_parameter('beta_madg', 0.1).value
        self.n_calib = self.declare_parameter('calib_samples', 300).value

        # ZUPT (zero velocity update): si detecta reposo, fuerza velocidad = 0
        self.zupt = self.declare_parameter('zupt', False).value
        self.zupt_gyro = self.declare_parameter('zupt_gyro', 0.05).value   # rad/s
        self.zupt_acc = self.declare_parameter('zupt_acc', 0.3).value      # m/s2
        self.still_count = 0

        self.plot = self.declare_parameter('plot', True).value
        self.plot_window = self.declare_parameter('plot_window_s', 20.0).value

        # Historial: (t, r0, r1, r2, p0, p1, p2, y0, y1, y2, z0, z1, z2) -> 13 variables
        self.hist = deque(maxlen=int(self.plot_window * 100))
        self.lock = threading.Lock()
        self.t_rel = 0.0

        self.count = 0
        self.sum_acc = np.zeros(3)
        self.sum_gyr = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.g = 9.80665

        # Estado para los 3 métodos: 0=DMP, 1=Comp, 2=Madgwick
        self.qs = [np.array([1.0, 0.0, 0.0, 0.0]) for _ in range(3)]
        self.vels = [np.zeros(3) for _ in range(3)]
        self.poss = [np.zeros(3) for _ in range(3)]
        self.t_prev = None

        self.tf_br = TransformBroadcaster(self)
        self.pub_markers = self.create_publisher(MarkerArray, '~/velocity_markers', 10)
        self.create_service(Empty, '~/reset', self.on_reset)
        self.create_subscription(Imu, self.topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info('Calibrando: dejar el sensor QUIETO unos segundos...')

    def reset(self):
        for i in range(3):
            self.vels[i][:] = 0.0
            self.poss[i][:] = 0.0
        self.get_logger().info('Velocidades y posiciones reiniciadas')

    def on_reset(self, request, response):
        self.reset()
        return response

    def on_imu(self, msg):
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ---- Calibración inicial ----
        if self.count < self.n_calib:
            self.sum_acc += acc
            self.sum_gyr += gyr
            self.count += 1
            self.t_prev = t
            if self.count == self.n_calib:
                mean_acc = self.sum_acc / self.n_calib
                self.gyro_bias = self.sum_gyr / self.n_calib
                self.g = float(np.linalg.norm(mean_acc))
                q_init = q_from_accel(mean_acc)
                for i in range(3):
                    self.qs[i] = q_init.copy()
                self.get_logger().info(
                    f'Listo. Bias gyro [rad/s]: {np.round(self.gyro_bias, 4)}, '
                    f'|g| medido: {self.g:.3f} m/s2')
            return

        dt = t - self.t_prev
        self.t_prev = t
        if not 0.0 < dt < 0.1:      # timestamp invalido o salto: usar periodo nominal
            dt = 0.01

        a_norm = np.linalg.norm(acc)
        w = gyr - self.gyro_bias

        # 0. Hardware DMP
        n_dmp = np.linalg.norm([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        if n_dmp > 1e-6:
            self.qs[0] = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]) / n_dmp

        # 1. Filtro Complementario (Mahony)
        if abs(a_norm - self.g) < 0.1 * self.g:
            up_est = q_to_rot(self.qs[1])[2, :]
            err = np.cross(acc / a_norm, up_est)
            w_comp = w + self.kp_comp * err
        else:
            w_comp = w
        dq_comp = 0.5 * q_mult(self.qs[1], np.array([0.0, w_comp[0], w_comp[1], w_comp[2]]))
        self.qs[1] = self.qs[1] + dq_comp * dt
        self.qs[1] /= np.linalg.norm(self.qs[1])

        # 2. Filtro de Madgwick
        s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0
        qw, qx, qy, qz = self.qs[2]
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
            0.5 * (-qx*w[0] - qy*w[1] - qz*w[2]) - self.beta_madg * s0,
            0.5 * ( qw*w[0] + qy*w[2] - qz*w[1]) - self.beta_madg * s1,
            0.5 * ( qw*w[1] - qx*w[2] + qz*w[0]) - self.beta_madg * s2,
            0.5 * ( qw*w[2] + qx*w[1] - qy*w[0]) - self.beta_madg * s3
        ])
        self.qs[2] = self.qs[2] + qDot * dt
        self.qs[2] /= np.linalg.norm(self.qs[2])

        # ---- Condición ZUPT general ----
        if self.zupt:
            still = (np.linalg.norm(w) < self.zupt_gyro and abs(a_norm - self.g) < self.zupt_acc)
            self.still_count = self.still_count + 1 if still else 0

        # ---- Cinemática para los 3 métodos ----
        eu = []
        for i in range(3):
            eu.append(np.degrees(q_to_euler(self.qs[i])))
            acc_world = q_to_rot(self.qs[i]) @ acc - np.array([0.0, 0.0, self.g])
            self.vels[i] += acc_world * dt
            if self.zupt and self.still_count >= 20:
                self.vels[i][:] = 0.0
            self.poss[i] += self.vels[i] * dt

        # Registro para gráficos
        self.t_rel += dt
        with self.lock:
            self.hist.append((
                self.t_rel,
                eu[0][0], eu[1][0], eu[2][0],  # Roll: DMP, Comp, Madg
                eu[0][1], eu[1][1], eu[2][1],  # Pitch
                eu[0][2], eu[1][2], eu[2][2],  # Yaw
                self.poss[0][2], self.poss[1][2], self.poss[2][2] # Altura Z
            ))

        self.publish_tfs()

    def publish_tfs(self):
        now = self.get_clock().now().to_msg()
        names = ['dmp', 'comp', 'madg']
        
        for i in range(3):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = 'odom'
            tf.child_frame_id = f'imu_link_{names[i]}'
            tf.transform.translation.x = float(self.poss[i][0])
            tf.transform.translation.y = float(self.poss[i][1])
            tf.transform.translation.z = float(self.poss[i][2])
            tf.transform.rotation.w = float(self.qs[i][0])
            tf.transform.rotation.x = float(self.qs[i][1])
            tf.transform.rotation.y = float(self.qs[i][2])
            tf.transform.rotation.z = float(self.qs[i][3])
            self.tf_br.sendTransform(tf)

        # Publicar las 3 flechas de velocidad
        ma = MarkerArray()
        # Colores RGB: DMP (Rojo), Comp (Verde), Madgwick (Azul)
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.5, 1.0)] 
        
        for i in range(3):
            mk = Marker()
            mk.header.stamp = now
            mk.header.frame_id = 'odom'
            mk.ns = f'vel_{names[i]}'
            mk.id = i
            mk.type = Marker.ARROW
            mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0
            
            # Origen en la posición estimada, fin en la posición + vector velocidad
            end = self.poss[i] + self.vels[i]
            mk.points = [
                Point(x=float(self.poss[i][0]), y=float(self.poss[i][1]), z=float(self.poss[i][2])),
                Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))
            ]
            mk.scale.x = 0.02   # diámetro del cuerpo de la flecha
            mk.scale.y = 0.04   # diámetro de la punta
            
            mk.color.r = colors[i][0]
            mk.color.g = colors[i][1]
            mk.color.b = colors[i][2]
            mk.color.a = 0.8    # Ligeramente transparente para que no se tapen si coinciden
            
            ma.markers.append(mk)

        self.pub_markers.publish(ma)

def run_plot(node):
    """Graficos en vivo. Bloquea hasta que se cierra la ventana."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    panels = [
        ('Roll [grados]',   (1, 2, 3)),
        ('Pitch [grados]',  (4, 5, 6)),
        ('Yaw [grados]',    (7, 8, 9)),
        ('Posición Z [m]',  (10, 11, 12)),
    ]
    labels = ['DMP', 'Comp', 'Madg']
    colors = ['r', 'g', 'b']

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    fig.suptitle('Comparación de Filtros IMU', fontsize=12)
    lines_list = []

    for ax, (title, idxs) in zip(axes, panels):
        ax.set_title(title, fontsize=10)
        ax.grid(True)
        ax_lines = []
        for i, idx in enumerate(idxs):
            line, = ax.plot([], [], label=labels[i], color=colors[i], alpha=0.8)
            ax_lines.append(line)
        lines_list.append((ax_lines, idxs))
        ax.legend(loc='upper left', ncol=3, fontsize=8)

    axes[-1].set_xlabel('Tiempo [s]     (tecla R: reiniciar velocidades y posiciones)')

    def on_key(event):
        if event.key in ('r', 'R'):
            node.reset()

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        with node.lock:
            data = np.array(node.hist)
        if len(data) < 2:
            return [l for ax_lines, _ in lines_list for l in ax_lines]
        
        t = data[:, 0]
        for ax, (ax_lines, idxs) in zip(axes, lines_list):
            for line, idx in zip(ax_lines, idxs):
                line.set_data(t, data[:, idx])
            ax.relim()
            ax.autoscale_view()
        axes[0].set_xlim(t[0], max(t[-1], t[0] + 1.0))
        return [l for ax_lines, _ in lines_list for l in ax_lines]

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
            run_plot(node)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()