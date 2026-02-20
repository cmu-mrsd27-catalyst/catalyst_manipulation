#!/usr/bin/env python3
"""
Plot grasp repeatability data from grasp_log.jsonl.

Reads the log file and produces:
1. Position scatter/line: x, y, z of computed grasp vs actual TCP over runs
2. Position error: Euclidean distance between computed and actual TCP per run
3. Tag pose stability: x, y, z of tag_object / tag_base across runs
"""

import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


def load_log(log_path):
    records = []
    with open(log_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def main():
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    log_path = os.path.join(log_dir, 'grasp_log.jsonl')

    if not os.path.exists(log_path):
        print(f'No log file found at {log_path}')
        sys.exit(1)

    records = load_log(log_path)
    if not records:
        print('Log file is empty')
        sys.exit(1)

    print(f'Loaded {len(records)} runs from {log_path}')

    runs = [r['run_id'] for r in records]

    # Extract poses
    computed = np.array([r['computed_grasp_pose'][:3] for r in records if r.get('computed_grasp_pose')])
    actual = np.array([r['actual_tcp_pose'][:3] for r in records if r.get('actual_tcp_pose')])

    tag_obj = []
    tag_obj_runs = []
    tag_base = []
    tag_base_runs = []
    for r in records:
        if r.get('tag_object_pose'):
            tag_obj.append(r['tag_object_pose'][:3])
            tag_obj_runs.append(r['run_id'])
        if r.get('tag_base_pose'):
            tag_base.append(r['tag_base_pose'][:3])
            tag_base_runs.append(r['run_id'])
    tag_obj = np.array(tag_obj) if tag_obj else None
    tag_base = np.array(tag_base) if tag_base else None

    fig, axes = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle('Grasp Repeatability Analysis', fontsize=14, fontweight='bold')

    # --- Plot 1: Computed vs Actual TCP position ---
    ax = axes[0]
    labels = ['x', 'y', 'z']
    colors = ['tab:red', 'tab:green', 'tab:blue']
    for i, (label, color) in enumerate(zip(labels, colors)):
        if len(computed) > 0:
            ax.plot(runs[:len(computed)], computed[:, i] * 1000, f'-o', color=color,
                    label=f'computed {label}', markersize=4)
        if len(actual) > 0:
            ax.plot(runs[:len(actual)], actual[:, i] * 1000, f'--s', color=color,
                    label=f'actual {label}', markersize=4, alpha=0.7)
    ax.set_xlabel('Run')
    ax.set_ylabel('Position (mm)')
    ax.set_title('Computed Grasp vs Actual TCP Position')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Plot 2: Position error ---
    ax = axes[1]
    n_paired = min(len(computed), len(actual))
    if n_paired > 0:
        errors = np.linalg.norm(computed[:n_paired] - actual[:n_paired], axis=1) * 1000
        ax.bar(runs[:n_paired], errors, color='tab:orange', alpha=0.8)
        ax.axhline(np.mean(errors), color='tab:red', linestyle='--',
                   label=f'mean = {np.mean(errors):.2f} mm')
        ax.set_ylabel('Error (mm)')
        ax.legend()
    else:
        ax.text(0.5, 0.5, 'No paired data available', transform=ax.transAxes,
                ha='center', va='center')
    ax.set_xlabel('Run')
    ax.set_title('Euclidean Position Error (Computed vs Actual)')
    ax.grid(True, alpha=0.3)

    # --- Plot 3: Tag pose stability ---
    ax = axes[2]
    if tag_obj is not None and len(tag_obj) > 0:
        for i, (label, color) in enumerate(zip(labels, colors)):
            ax.plot(tag_obj_runs, tag_obj[:, i] * 1000, f'-o', color=color,
                    label=f'tag_object {label}', markersize=4)
    if tag_base is not None and len(tag_base) > 0:
        for i, (label, color) in enumerate(zip(labels, colors)):
            ax.plot(tag_base_runs, tag_base[:, i] * 1000, f'--^', color=color,
                    label=f'tag_base {label}', markersize=4, alpha=0.7)
    ax.set_xlabel('Run')
    ax.set_ylabel('Position (mm)')
    ax.set_title('AprilTag Detection Stability')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    out_path = os.path.join(log_dir, 'grasp_analysis.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved plot to {out_path}')
    plt.show()


if __name__ == '__main__':
    main()
