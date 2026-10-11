#!/usr/bin/env python3
"""Tabla de error de cada metodo contra el encoder, a partir de errores_encoder.csv.

    python3 tabla_error.py                 todos los ensayos juntos
    python3 tabla_error.py -e 3            solo el ensayo 3 (se puede pasar varios: -e 3 4 5)
    python3 tabla_error.py --por-ensayo    una tabla por ensayo
    python3 tabla_error.py --md            en Markdown (para el informe)

error = angulo estimado - angulo del encoder (llevado a +-180 grados)
  RMSE        raiz del promedio de error^2: el error tipico (pesa mas los errores grandes)
  Error max   el peor |error| del ensayo
  Error medio promedio del error con signo: si no da ~0, el metodo tiene un sesgo
"""
import argparse
import csv
from collections import defaultdict

import numpy as np

from tabla_calibracion import tabla

NOMBRES = {'complementary_estimator': 'Complementario', 'dmp_estimator': 'DMP',
           'madgwick_estimator': 'Madgwick'}


def sentido(datos):
    """Signo del encoder (+1/-1) que mejor coincide con los metodos."""
    enc = np.concatenate([d[:, 0] for d in datos.values()])
    est = np.concatenate([d[:, 1] for d in datos.values()])
    return 1 if np.sum(enc * est) >= 0 else -1


def tabla_metodos(datos, sign, md):
    filas = []
    for m, d in sorted(datos.items(), key=lambda x: NOMBRES.get(x[0], x[0])):
        err = (d[:, 1] - sign * d[:, 0] + 180) % 360 - 180
        filas.append([NOMBRES.get(m, m), str(len(err)), f'{np.max(np.abs(sign * d[:, 0])):.1f}',
                      f'{np.sqrt(np.mean(err ** 2)):.2f}', f'{np.max(np.abs(err)):.2f}',
                      f'{np.mean(err):+.2f}'])
    return tabla(['Metodo', 'Muestras', 'Giro max [°]', 'RMSE [°]', 'Error max [°]', 'Error medio [°]'],
                 filas, md)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('archivo', nargs='?', default='errores_encoder.csv')
    ap.add_argument('-e', '--ensayo', type=int, nargs='+', help='ensayos a incluir (por defecto todos)')
    ap.add_argument('--por-ensayo', action='store_true', help='una tabla por cada ensayo')
    ap.add_argument('--md', action='store_true', help='salida en Markdown')
    args = ap.parse_args()

    # ensayo -> metodo -> filas (encoder_deg, estimado_deg)
    datos = defaultdict(lambda: defaultdict(list))
    with open(args.archivo, newline='') as f:
        for r in csv.DictReader(f):
            e = int(r['ensayo'])
            if args.ensayo is None or e in args.ensayo:
                datos[e][r['metodo']].append((float(r['encoder_deg']), float(r['estimado_deg'])))
    if not datos:
        print('No hay datos para esos ensayos en', args.archivo)
        return

    todos = defaultdict(list)
    for por_metodo in datos.values():
        for m, filas in por_metodo.items():
            todos[m] += filas
    todos = {m: np.array(f) for m, f in todos.items()}
    sign = sentido(todos)          # el sentido es fisico: el mismo en todos los ensayos

    sep = '' if args.md else '\n'
    if args.por_ensayo:
        for e in sorted(datos):
            print(f'\nEnsayo {e}{sep}')
            print(tabla_metodos({m: np.array(f) for m, f in datos[e].items()}, sign, args.md))
    else:
        ens = ', '.join(map(str, sorted(datos)))
        print(f'Error contra el encoder (ensayos {ens}){sep}')
        print(tabla_metodos(todos, sign, args.md))
    print(f'\nSentido del encoder usado: {sign:+d}')


if __name__ == '__main__':
    main()
