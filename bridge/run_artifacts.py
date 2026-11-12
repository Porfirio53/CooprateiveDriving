"""Standalone trajectory figures with the actual static obstacle footprints."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import pandas as pd


def plot_trajectory(directory):
    directory = Path(directory)
    reference_file = directory / 'scoring_reference.csv'
    if not reference_file.exists():
        return
    reference = np.atleast_2d(np.loadtxt(reference_file, delimiter=',', skiprows=1))
    fig, axis = plt.subplots(figsize=(9, 7), constrained_layout=True)
    axis.plot(reference[:, 0], reference[:, 1], '--', color='0.45', linewidth=1.5, label='Reference')
    trajectory_path = directory / 'trajectory.csv'
    if trajectory_path.exists() and trajectory_path.stat().st_size:
        trajectory = pd.read_csv(trajectory_path)
        scored = trajectory[trajectory.phase == 'score']
        if len(scored):
            axis.plot(scored.x_m, scored.y_m, color='#0072B2', linewidth=1.5, label='Vehicle')
            axis.scatter(scored.x_m.iloc[-1], scored.y_m.iloc[-1], c='#0072B2', marker='x', zorder=4)
    scene_path = directory / 'scene.json'
    if scene_path.exists():
        scene = json.loads(scene_path.read_text())
        for i, obstacle in enumerate(scene['obstacles']):
            heading = np.deg2rad(obstacle['yaw'])
            rotation = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
            ex, ey = obstacle['extent_x_m'], obstacle['extent_y_m']
            corners = np.array([[-ex, -ey], [ex, -ey], [ex, ey], [-ex, ey]])
            corners += [obstacle['box_offset_x_m'], obstacle['box_offset_y_m']]
            corners = corners @ rotation.T + [obstacle['x'], obstacle['y']]
            axis.add_patch(Polygon(corners, color='#D55E00', alpha=.8, label='Parked obstacle' if i == 0 else None))
    config = json.loads((directory / 'config.json').read_text())
    axis.scatter(*reference[0, :2], c='#009E73', marker='o', label='Start', zorder=4)
    axis.scatter(*reference[-1, :2], c='#CC79A7', marker='*', s=80, label='Finish', zorder=4)
    axis.set(xlabel='CARLA world x (m)', ylabel='CARLA world y (m)',
             title=f"{config.get('map', 'Town04')} · {config['condition']} · {config['participant']}")
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(alpha=.2)
    axis.legend()
    fig.savefig(directory / 'trajectory.png', dpi=180)
    fig.savefig(directory / 'trajectory.svg')
    plt.close(fig)
