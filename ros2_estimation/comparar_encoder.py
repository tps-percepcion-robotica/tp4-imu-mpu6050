#!/usr/bin/env python3
"""Compara el angulo del disco medido por el encoder AS5600 con el que estima cada metodo.

El MPU-6050 esta pegado sobre la cara del disco, asi que al girarlo el sensor rota
alrededor de su eje z. Para cada metodo se toma la rotacion respecto de la orientacion
al inicio del ensayo y se extrae su componente alrededor de z:

    q_rel = q_inicial^-1 (x) q        theta = 2 * atan2(q_rel.z, q_rel.w)

(no se usan roll/pitch/yaw porque con el eje horizontal quedan cerca de la singularidad).
El encoder y la IMU salen del ESP32 con el mismo timestamp, y los estimadores publican la
odometria con ese timestamp, asi que cada estimacion se compara con la lectura del encoder
del mismo instante.

Cada muestra se guarda en errores_encoder.csv; tabla_error.py calcula RMSE / maximo / medio.
Tecla N en el grafico (o servicio ~/nuevo_ensayo): nuevo ensayo, todo vuelve a 0.
"""
import csv
import math
import os
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty

from imu_state_estimator import q_conj, q_mult

NOMBRES = {'complementary_estimator': 'Complementario', 'dmp_estimator': 'DMP',
           'madgwick_estimator': 'Madgwick'}
# Grafico tipo reloj: etiqueta, color y largo de cada aguja (distinto largo para ver las superpuestas)
AGUJAS = {'dmp_estimator': ('dmp', 'tab:blue', 0.85),
          'complementary_estimator': ('comp', 'tab:orange', 0.70),
          'madgwick_estimator': ('madg', 'tab:green', 0.55)}
POS_VENTANA = 20.0      # segundos de posicion que se muestran
POS_PERIODO = 0.02      # se guarda una muestra de posicion cada 20 ms


def wrap(a):
    """Lleva un angulo [rad] a (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ComparadorEncoder(Node):
    def __init__(self):
        super().__init__('comparador_encoder')
        self.metodos = self.declare_parameter(
            'metodos', list(NOMBRES)).value
        enc_topic = self.declare_parameter('encoder_topic', 'encoder/angle').value
        self.csv_path = self.declare_parameter('csv', 'errores_encoder.csv').value
        # Sentido del encoder respecto del eje z de la IMU: 1, -1, o 0 = detectarlo solo
        self.sign = self.declare_parameter('encoder_sign', 0).value
        self.sign_auto = self.sign == 0
        if self.sign_auto:
            self.sign = 1
        self.plot = self.declare_parameter('plot', True).value

        self.lock = threading.Lock()
        self.enc = {}                   # timestamp [ns] -> angulo crudo del encoder [rad]
        self.enc_keys = deque()
        self.sin_par = 0                # estimaciones sin lectura de encoder del mismo instante

        self.ensayo = self.ultimo_ensayo()
        self.nuevo_ensayo()

        nuevo = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv = csv.writer(self.csv_file)
        if nuevo:
            self.csv.writerow(['ensayo', 'metodo', 't_s', 'encoder_deg', 'estimado_deg'])
        self.create_timer(1.0, self.csv_file.flush)

        self.create_subscription(Vector3Stamped, enc_topic, self.on_encoder, qos_profile_sensor_data)
        for m in self.metodos:
            self.create_subscription(Odometry, f'/{m}/odom',
                                     lambda msg, m=m: self.on_odom(m, msg), 50)
        self.create_service(Empty, '~/nuevo_ensayo', self.on_nuevo_ensayo)
        self.get_logger().info(f'Comparando {", ".join(self.metodos)} contra {enc_topic}. '
                               f'Ensayo {self.ensayo} (tecla N: nuevo ensayo)')

    def ultimo_ensayo(self):
        """Sigue la numeracion de ensayos de un CSV anterior."""
        try:
            with open(self.csv_path, newline='') as f:
                return max((int(r['ensayo']) for r in csv.DictReader(f)), default=0)
        except (OSError, KeyError, ValueError):
            return 0

    def nuevo_ensayo(self):
        """Toma la proxima muestra de cada metodo como cero (orientacion y encoder)."""
        with self.lock:
            self.ensayo += 1
            self.q0 = {m: None for m in self.metodos}
            self.enc0 = {m: None for m in self.metodos}
            self.t0 = None
            self.ultimo = {m: None for m in self.metodos}   # (encoder, estimado) [rad] mas reciente
            # Pares (encoder, estimado) [rad] del ensayo, para detectar el sentido del encoder
            self.pares = {m: [] for m in self.metodos}
            # Posicion calculada por cada metodo: (t, x, y, z), y su ultima velocidad
            self.t0_pos = None
            self.pos_hist = {m: deque(maxlen=int(POS_VENTANA / POS_PERIODO)) for m in self.metodos}
            self.vel = {m: (0.0, 0.0, 0.0) for m in self.metodos}

    def on_nuevo_ensayo(self, request, response):
        self.nuevo_ensayo()
        self.get_logger().info(f'Ensayo {self.ensayo}: dejar el disco quieto, todo vuelve a 0')
        return response

    def on_encoder(self, msg):
        k = stamp_ns(msg.header.stamp)
        self.enc[k] = msg.vector.x
        self.enc_keys.append(k)
        while len(self.enc_keys) > 4000:
            self.enc.pop(self.enc_keys.popleft(), None)

    def on_odom(self, m, msg):
        k = stamp_ns(msg.header.stamp)
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:                 # la posicion se grafica aunque no haya lectura del encoder
            self.vel[m] = (v.x, v.y, v.z)
            if self.t0_pos is None:
                self.t0_pos = k
            tp = (k - self.t0_pos) * 1e-9
            h = self.pos_hist[m]
            if not h or tp - h[-1][0] >= POS_PERIODO:
                h.append((tp, p.x, p.y, p.z))

        enc = self.enc.get(k)
        if enc is None:
            self.sin_par += 1
            return
        o = msg.pose.pose.orientation
        q = np.array([o.w, o.x, o.y, o.z])

        with self.lock:
            if self.q0[m] is None:      # primera muestra del ensayo: es el cero
                self.q0[m] = q
                self.enc0[m] = enc
                if self.t0 is None:
                    self.t0 = k
            q_rel = q_mult(q_conj(self.q0[m]), q)
            est = wrap(2 * math.atan2(q_rel[3], q_rel[0]))
            enc_rel = wrap(enc - self.enc0[m])
            t = (k - self.t0) * 1e-9
            self.pares[m].append((enc_rel, est))
            self.ultimo[m] = (enc_rel, est)
            if self.sign_auto:
                self.detectar_sentido()
            self.csv.writerow([self.ensayo, m, f'{t:.4f}', f'{math.degrees(enc_rel):.3f}',
                               f'{math.degrees(est):.3f}'])

    def detectar_sentido(self):
        """Elige el signo del encoder que mejor coincide con los metodos, una vez que giro >15 grados."""
        e = np.array([p for ps in self.pares.values() for p in ps[-500:]])
        if len(e) and np.max(np.abs(e[:, 0])) > math.radians(15):
            sign = 1 if np.sum(e[:, 0] * e[:, 1]) >= 0 else -1
            self.sign_auto = False
            self.sign = sign
            self.get_logger().info(f'Sentido del encoder detectado: {sign:+d} '
                                   f'(fijarlo con -p encoder_sign:={sign})')


def run_plot(node):
    """Arriba a la izquierda: una aguja por metodo y el encoder (angulo del disco).
    Arriba a la derecha: el error de cada metodo en escala real (1 grado de error = 1 grado en
    el circulo), con el valor en numeros al lado.
    Derecha: trayectoria 3D de la posicion calculada por cada metodo, con |p| y |v| actuales."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(17, 6))
    ax_ang = fig.add_subplot(1, 3, 1, projection='polar')
    ax_err = fig.add_subplot(1, 3, 2, projection='polar')
    ax_tray = fig.add_subplot(1, 3, 3, projection='3d')
    ax_tray.set_title('Trayectoria estimada (posicion)', pad=20)
    ax_tray.set_xlabel('X [m]')
    ax_tray.set_ylabel('Y [m]')
    ax_tray.set_zlabel('Z [m]')
    ax_tray.set_box_aspect((1, 1, 1))
    ax_tray.text2D(0.0, 1.0, f'{"":6}{"|p| [m]":>9}{"|v| [m/s]":>11}', transform=ax_tray.transAxes,
                   family='monospace', fontsize=9)
    textos_tray = {}
    for ax in (ax_ang, ax_err):
        ax.set_theta_zero_location('N')     # 0 arriba; los angulos positivos giran a la izquierda
        ax.set_ylim(0, 1)
        ax.set_yticks([])

    ax_ang.set_title('Giro en Z: vuelta completa', pad=20)
    ax_ang.set_thetagrids(range(0, 360, 45), [f'{a}°' for a in range(0, 360, 45)])
    ax_err.set_title('Error respecto del encoder\n(izquierda: el metodo marca de mas)', pad=20)
    ax_err.set_thetagrids(range(0, 360, 45),
                          ['0°', '+45°', '+90°', '+135°', '±180°', '-135°', '-90°', '-45°'])
    textos_err = {}
    lineas_pos = {}

    def aguja(ax, color, largo, lw):
        return ax.plot([0, 0], [0, largo], color=color, lw=lw, marker='o', markevery=[1])[0]

    enc_ang = aguja(ax_ang, 'black', 0.95, 4)
    enc_err = aguja(ax_err, 'black', 0.95, 4)
    agujas_ang, agujas_err = {}, {}
    for m in node.metodos:
        etiqueta, color, largo = AGUJAS.get(m, (m, None, 0.6))
        agujas_ang[m] = aguja(ax_ang, color, largo, 2.5)
        agujas_err[m] = aguja(ax_err, color, largo, 2.5)
        textos_err[m] = ax_err.text(0.02, 0.0, '', transform=ax_err.transAxes, color=color,
                                    fontsize=11, fontweight='bold')
        agujas_ang[m].set_label(etiqueta)
        lineas_pos[m] = (ax_tray.plot([], [], [], color=color, lw=1.5)[0],                 # recorrido
                         ax_tray.plot([], [], [], color=color, marker='o', ls='')[0])     # ahora
    enc_ang.set_label('enc')
    orden = sorted(agujas_ang, key=lambda m: list(AGUJAS).index(m) if m in AGUJAS else len(AGUJAS))
    for i, m in enumerate(orden):            # valores del error, uno debajo del otro
        textos_err[m].set_position((1.02, 0.95 - 0.08 * i))
        textos_tray[m] = ax_tray.text2D(0.0, 0.95 - 0.05 * i, '', transform=ax_tray.transAxes,
                                        family='monospace', fontsize=9, color=AGUJAS[m][1])
    fig.legend(handles=[enc_ang, *(agujas_ang[m] for m in orden)], loc='upper left')
    fig.subplots_adjust(top=0.82, bottom=0.10, left=0.04, right=0.97, wspace=0.30)
    pie = fig.text(0.5, 0.02, '', ha='center')

    def on_key(event):
        if event.key in ('n', 'N'):
            node.on_nuevo_ensayo(None, None)
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        with node.lock:
            ultimo = dict(node.ultimo)
            sign = node.sign
            pos = {m: np.array(node.pos_hist[m]) for m in node.metodos}
            vel = dict(node.vel)
        pie.set_text(f'Ensayo {node.ensayo}     (tecla N: nuevo ensayo, todo vuelve a 0)')
        for m in node.metodos:
            if ultimo[m] is None:
                continue
            enc, est = sign * ultimo[m][0], ultimo[m][1]
            th = [est, est]
            agujas_ang[m].set_xdata(th)
            enc_ang.set_xdata([enc, enc])
            err = wrap(est - enc)
            agujas_err[m].set_xdata([err, err])
            textos_err[m].set_text(f'{AGUJAS.get(m, (m,))[0]}: {math.degrees(err):+.1f}°')

        puntos = [np.zeros((1, 3))]          # el origen siempre queda a la vista
        for m in node.metodos:
            d = pos[m]
            if len(d) < 1:
                continue
            xyz = d[:, 1:4]
            puntos.append(xyz)
            recorrido, ahora = lineas_pos[m]
            recorrido.set_data_3d(xyz[:, 0], xyz[:, 1], xyz[:, 2])
            ahora.set_data_3d(xyz[-1:, 0], xyz[-1:, 1], xyz[-1:, 2])
            textos_tray[m].set_text(f'{AGUJAS[m][0]:6}{np.linalg.norm(xyz[-1]):9.2f}'
                                    f'{np.linalg.norm(vel[m]):11.2f}')
        # Mismo rango en los tres ejes, para no deformar la trayectoria
        todos = np.vstack(puntos)
        centro = (todos.max(axis=0) + todos.min(axis=0)) / 2
        r = max(np.max(todos.max(axis=0) - todos.min(axis=0)) / 2, 0.05)
        ax_tray.set_xlim(centro[0] - r, centro[0] + r)
        ax_tray.set_ylim(centro[1] - r, centro[1] + r)
        ax_tray.set_zlim(centro[2] - r, centro[2] + r)
        return []

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()


def spin_node(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'El nodo se detuvo: {e!r}')


def main():
    rclpy.init()
    node = ComparadorEncoder()
    try:
        if node.plot:
            threading.Thread(target=spin_node, args=(node,), daemon=True).start()
            run_plot(node)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
