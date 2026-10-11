#!/usr/bin/env python3
"""Una sola ventana para comparar los tres metodos entre si.

Grilla de 4 filas (actitud, aceleracion en el mundo, velocidad, posicion) por 3 columnas
(roll/x, pitch/y, yaw/z). Cada grafico superpone los tres metodos, y los tres graficos de
una fila comparten la escala vertical, asi la comparacion es directa.

Escucha /<metodo>/odom (actitud, velocidad, posicion) y /<metodo>/accel_world.
Tecla R: recalibra los tres metodos a la vez (dejar el sensor quieto ~2 s).
Tecla G: re-mide la gravedad en la posicion actual, sin tocar orientacion ni posicion
         (sirve cuando, en una posicion del sensor donde el acelerometro mide otro valor
         de |g|, la velocidad crece sola aunque este quieto).
"""
import math
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty

from comparar_encoder import AGUJAS
from imu_state_estimator import q_to_euler

METODOS = list(AGUJAS)          # dmp, comp, madg (mismos colores que el reloj del encoder)
FILAS = [('Actitud [grados]', ('roll', 'pitch', 'yaw')),
         ('Aceleracion en el mundo [m/s2]', ('x', 'y', 'z')),
         ('Velocidad [m/s]', ('x', 'y', 'z')),
         ('Posicion [m]', ('x', 'y', 'z'))]
PERIODO_GRAFICO = 0.02          # se guarda una muestra cada 20 ms (50 Hz) para no cargar el grafico


class ComparadorMetodos(Node):
    def __init__(self):
        super().__init__('comparador_metodos')
        self.window = self.declare_parameter('plot_window_s', 20.0).value
        self.lock = threading.Lock()
        self.acc = {m: (0.0, 0.0, 0.0) for m in METODOS}
        self.reset_hist()

        self.resets = {m: self.create_client(Empty, f'/{m}/reset') for m in METODOS}
        self.remedir = {m: self.create_client(Empty, f'/{m}/remedir_gravedad') for m in METODOS}
        for m in METODOS:
            self.create_subscription(Odometry, f'/{m}/odom', lambda msg, m=m: self.on_odom(m, msg), 50)
            self.create_subscription(Vector3Stamped, f'/{m}/accel_world',
                                     lambda msg, m=m: self.on_acc(m, msg), 50)
        self.get_logger().info('Comparando los tres metodos en una sola ventana '
                               '(tecla R: recalibrar, tecla G: remedir gravedad)')

    def reset_hist(self):
        with self.lock:
            self.t0 = None
            # Por metodo: (t, roll, pitch, yaw, ax, ay, az, vx, vy, vz, px, py, pz)
            self.hist = {m: deque(maxlen=int(self.window / PERIODO_GRAFICO)) for m in METODOS}

    def recalibrar(self):
        """Pide a los tres estimadores que recalibren a la vez."""
        for m, cli in self.resets.items():
            if cli.service_is_ready():
                cli.call_async(Empty.Request())
            else:
                self.get_logger().warn(f'{m} no responde al pedido de recalibracion')
        self.reset_hist()
        self.get_logger().info('Recalibrando los tres metodos: dejar el sensor QUIETO ~2 s')

    def remedir_gravedad(self):
        """Pide a los tres estimadores que vuelvan a medir |g| en la posicion actual,
        sin tocar orientacion ni posicion (no llama a reset_hist: los graficos siguen)."""
        for m, cli in self.remedir.items():
            if cli.service_is_ready():
                cli.call_async(Empty.Request())
            else:
                self.get_logger().warn(f'{m} no responde al pedido de remedir gravedad')
        self.get_logger().info('Remidiendo gravedad en esta posicion: dejar el sensor QUIETO')

    def on_acc(self, m, msg):
        v = msg.vector
        self.acc[m] = (v.x, v.y, v.z)

    def on_odom(self, m, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        o, p, v = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist.linear
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            t -= self.t0
            h = self.hist[m]
            if h and t - h[-1][0] < PERIODO_GRAFICO:
                return
            euler = [math.degrees(a) for a in q_to_euler((o.w, o.x, o.y, o.z))]
            h.append((t, *euler, *self.acc[m], v.x, v.y, v.z, p.x, p.y, p.z))


def run_plot(node):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, axes = plt.subplots(4, 3, sharex=True, sharey='row', figsize=(13, 9))
    lineas = {m: [] for m in METODOS}
    for f, (titulo, ejes) in enumerate(FILAS):
        axes[f, 0].set_ylabel(titulo, fontsize=9)
        for c, eje in enumerate(ejes):
            ax = axes[f, c]
            ax.grid(True)
            ax.set_title(eje, fontsize=9)
            for m in METODOS:
                etiqueta, color, _ = AGUJAS[m]
                lineas[m].append(ax.plot([], [], color=color, lw=1.2, label=etiqueta)[0])
    for c in range(3):
        axes[-1, c].set_xlabel('Tiempo [s]')
    fig.legend(handles=[lineas[m][0] for m in METODOS], loc='upper right', ncol=3)
    fig.suptitle('Comparacion de los tres metodos     '
                '(tecla R: recalibrar los tres, tecla G: remedir gravedad)', fontsize=12)

    def on_key(event):
        if event.key in ('r', 'R'):
            node.recalibrar()
        elif event.key in ('g', 'G'):
            node.remedir_gravedad()
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame):
        with node.lock:
            datos = {m: np.array(node.hist[m]) for m in METODOS}
        t_max = 0.0
        for m in METODOS:
            d = datos[m]
            if len(d) < 2:
                continue
            t_max = max(t_max, d[-1, 0])
            for i, linea in enumerate(lineas[m]):
                linea.set_data(d[:, 0], d[:, i + 1])
        for ax in axes.flat:
            ax.relim()
            ax.autoscale_view()
        axes[0, 0].set_xlim(max(0.0, t_max - node.window), max(t_max, 1.0))
        return []

    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    plt.show()


def spin_node(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'El nodo se detuvo: {e!r}')


def main():
    rclpy.init()
    node = ComparadorMetodos()
    try:
        threading.Thread(target=spin_node, args=(node,), daemon=True).start()
        run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
