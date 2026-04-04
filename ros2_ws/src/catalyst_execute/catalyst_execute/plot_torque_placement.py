#!/usr/bin/env python3
"""Visualize F/T data from torque placement test.

Plots 6 subplots showing Fx, Fy, Fz, Tx, Ty, Tz vs time.
Highlights CORRECT phases in red background.

Usage:
  python3 plot_torque_placement.py <csv_file>
  python3 plot_torque_placement.py  # uses latest file in logs/
"""

import csv
import glob
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            row = {}
            for k, v in r.items():
                try:
                    row[k] = float(v)
                except ValueError:
                    row[k] = v
            rows.append(row)
    return rows


def find_latest_log():
    log_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'logs'
    )
    files = glob.glob(os.path.join(log_dir, 'torque_placement_*.csv'))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def plot_ft_data(csv_path):
    rows = load_csv(csv_path)
    if not rows:
        print('No data in file.')
        return

    t = np.array([r['time'] for r in rows])
    fx = np.array([r['fx'] for r in rows])
    fy = np.array([r['fy'] for r in rows])
    fz = np.array([r['fz'] for r in rows])
    tx = np.array([r['tx'] for r in rows])
    ty = np.array([r['ty'] for r in rows])
    tz = np.array([r['tz'] for r in rows])

    # Find phase regions for highlighting
    def find_phase_regions(rows, t, phase_name):
        regions = []
        in_phase = False
        start_t = 0.0
        for r in rows:
            if r.get('phase') == phase_name and not in_phase:
                start_t = r['time']
                in_phase = True
            elif r.get('phase') != phase_name and in_phase:
                regions.append((start_t, r['time']))
                in_phase = False
        if in_phase:
            regions.append((start_t, t[-1]))
        return regions

    search_y_regions = find_phase_regions(rows, t, 'SEARCH_Y')
    search_z_regions = find_phase_regions(rows, t, 'SEARCH_Z')

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle(f'F/T Data: {os.path.basename(csv_path)}', fontsize=14)

    data = [
        (axes[0, 0], fx, 'Fx (N)', 'tab:blue'),
        (axes[1, 0], fy, 'Fy (N)', 'tab:orange'),
        (axes[2, 0], fz, 'Fz (N)', 'tab:green'),
        (axes[0, 1], tx, 'Tx (Nm)', 'tab:red'),
        (axes[1, 1], ty, 'Ty (Nm)', 'tab:purple'),
        (axes[2, 1], tz, 'Tz (Nm)', 'tab:brown'),
    ]

    for ax, values, label, color in data:
        for t_start, t_end in search_y_regions:
            ax.axvspan(t_start, t_end, alpha=0.2, color='orange')
        for t_start, t_end in search_z_regions:
            ax.axvspan(t_start, t_end, alpha=0.2, color='red')

        ax.plot(t, values, color=color, linewidth=0.8)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linewidth=0.5)

    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 1].set_xlabel('Time (s)')

    # Legend
    descend_patch = mpatches.Patch(color='white', label='DESCEND')
    search_y_patch = mpatches.Patch(color='orange', alpha=0.2, label='SEARCH_Y')
    search_z_patch = mpatches.Patch(color='red', alpha=0.2, label='SEARCH_Z')
    fig.legend(handles=[descend_patch, search_y_patch, search_z_patch], loc='upper right')

    plt.tight_layout()

    # Save next to CSV
    png_path = csv_path.replace('.csv', '.png')
    plt.savefig(png_path, dpi=150)
    print(f'Saved plot to {png_path}')

    plt.show()


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = find_latest_log()
        if csv_path is None:
            print('No log files found. Pass a CSV path as argument.')
            return

    print(f'Plotting: {csv_path}')
    plot_ft_data(csv_path)


if __name__ == '__main__':
    main()
