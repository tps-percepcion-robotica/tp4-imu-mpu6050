#!/usr/bin/env python3
"""Muestra como tabla las calibraciones guardadas por imu_state_estimator.py.

    python3 tabla_calibracion.py                  tabla en la terminal
    python3 tabla_calibracion.py --md             tabla en Markdown (para el informe)
    python3 tabla_calibracion.py -n 6             solo las ultimas 6 calibraciones
    python3 tabla_calibracion.py otro.csv         leer otro archivo
"""
import argparse
import csv
import math
import statistics
from datetime import datetime

G_REF = 9.80665            # gravedad estandar [m/s2]
NOMBRES = {'DMP': 'DMP', 'madgwick': 'Madgwick', 'complementary': 'Complementario'}


def tabla(encabezado, filas, md):
    """Devuelve la tabla como texto, alineada a la derecha salvo la columna 'Metodo'."""
    if md:
        esc = lambda f: [str(x).replace('|', '\\|') for x in f]   # '|g|' rompe la tabla
        encabezado, filas = esc(encabezado), [esc(f) for f in filas]
        lineas = ['| ' + ' | '.join(encabezado) + ' |',
                  '|' + '|'.join('---' if c in ('Metodo', 'Magnitud') else '---:' for c in encabezado) + '|']
        lineas += ['| ' + ' | '.join(f) + ' |' for f in filas]
        return '\n'.join(lineas)
    anchos = [max(len(str(x)) for x in col) for col in zip(encabezado, *filas)]

    def linea(f):
        return '  '.join(str(x).ljust(a) if encabezado[i] in ('Metodo', 'Magnitud') else str(x).rjust(a)
                         for i, (x, a) in enumerate(zip(f, anchos)))
    return '\n'.join([linea(encabezado), '  '.join('-' * a for a in anchos)] + [linea(f) for f in filas])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('archivo', nargs='?', default='calibraciones.csv')
    ap.add_argument('--md', action='store_true', help='salida en Markdown')
    ap.add_argument('-n', type=int, default=0, help='mostrar solo las ultimas N filas')
    args = ap.parse_args()

    with open(args.archivo, newline='') as f:
        datos = list(csv.DictReader(f))
    if args.n > 0:
        datos = datos[-args.n:]
    if not datos:
        print('No hay calibraciones en', args.archivo)
        return

    t0 = datetime.fromisoformat(datos[0]['fecha'])
    filas = []
    for i, d in enumerate(datos, 1):
        t = (datetime.fromisoformat(d['fecha']) - t0).total_seconds()
        filas.append([str(i), d['fecha'][11:], f'{t:.0f}', NOMBRES.get(d['metodo'], d['metodo']),
                      d['frecuencia_hz'],
                      f"{float(d['bias_gx_rad_s']):.4f}", f"{float(d['bias_gy_rad_s']):.4f}",
                      f"{float(d['bias_gz_rad_s']):.4f}", f"{float(d['g_medido_m_s2']):.3f}"])

    print('Calibraciones del sensor' + ('' if args.md else '\n'))
    print(tabla(['#', 'Hora', 't [s]', 'Metodo', 'Frec [Hz]',
                 'Bias gx [rad/s]', 'Bias gy [rad/s]', 'Bias gz [rad/s]', '|g| [m/s2]'], filas, args.md))

    # Resumen: promedio, desvio y variacion maxima de cada magnitud
    resumen = []
    for nombre, col, grados in [('Bias gx', 'bias_gx_rad_s', True), ('Bias gy', 'bias_gy_rad_s', True),
                                ('Bias gz', 'bias_gz_rad_s', True), ('|g|', 'g_medido_m_s2', False)]:
        v = [float(d[col]) for d in datos]
        prom = statistics.mean(v)
        desv = statistics.stdev(v) if len(v) > 1 else 0.0
        unidad = 'rad/s' if grados else 'm/s2'
        extra = (f'{math.degrees(prom):.2f} °/s' if grados
                 else f'{100 * (prom - G_REF) / G_REF:+.1f} % vs {G_REF}')
        resumen.append([nombre, f'{prom:.4f} {unidad}', f'{desv:.4f}', f'{max(v) - min(v):.4f}', extra])
    print('\nResumen' + ('' if args.md else '\n'))
    print(tabla(['Magnitud', 'Promedio', 'Desvio', 'Variacion max', 'Equivalente'], resumen, args.md))


if __name__ == '__main__':
    main()
