#!/usr/bin/env python3
"""
Overlay all waypoint scan captures in the global (odom) frame.

Reads all wp_*.yaml + .npy files from data/captures/, transforms each
scan from robot-local (base_scan) coordinates to global (odom) coordinates
using the saved pose, and plots them all overlaid.

Usage (from proj6_ws/src):
    python3 analysis/visualize_captures.py
    python3 analysis/visualize_captures.py --measure          # interactive measurement mode
    python3 analysis/visualize_captures.py --captures-dir data/captures
    python3 analysis/visualize_captures.py --output figures/overlay.png
"""

import argparse
import math
import os
import glob

import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.cm as cm


FEATURES = ['North Wall', 'East Door', 'Bin Corner']
DOT_SIZE = 12        # scatter point size
DOT_SIZE_DIM = 12   # size for dimmed background scans in measure mode


def load_capture(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    npy_path = os.path.join(os.path.dirname(yaml_path), meta['scan']['ranges_file'])
    ranges = np.load(npy_path)
    return meta, ranges


def scan_to_global_points(meta, ranges):
    pose = meta['pose']
    rx, ry, yaw = pose['x'], pose['y'], pose['yaw_rad']

    scan = meta['scan']
    angle_min = scan['angle_min']
    angle_increment = scan['angle_increment']
    range_max = scan['range_max']

    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges < range_max)

    r = ranges[valid]
    a = angles[valid]

    lx = r * np.cos(a)
    ly = r * np.sin(a)

    gx = rx + lx * math.cos(yaw) - ly * math.sin(yaw)
    gy = ry + lx * math.sin(yaw) + ly * math.cos(yaw)

    return gx, gy


def draw_robot(ax, meta, color):
    rx, ry = meta['pose']['x'], meta['pose']['y']
    yaw = meta['pose']['yaw_rad']
    ax.plot(rx, ry, 'o', color=color, markersize=8,
            markeredgecolor='black', linewidth=0.5, zorder=5)
    arrow_len = 0.15
    ax.annotate('',
                xy=(rx + arrow_len * math.cos(yaw), ry + arrow_len * math.sin(yaw)),
                xytext=(rx, ry),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5))


def build_overlay(ax, captures, colors):
    """Plot all captures overlaid (normal view)."""
    for color, (wp_id, meta, gx, gy) in zip(colors, captures):
        label = f'WP {wp_id}'
        ax.scatter(gx, gy, s=DOT_SIZE, color=color, alpha=0.7, label=label)
        rx, ry = meta['pose']['x'], meta['pose']['y']
        draw_robot(ax, meta, color)
        ax.annotate(str(wp_id), (rx, ry), textcoords='offset points',
                    xytext=(5, 5), fontsize=9, color=color, fontweight='bold')
    ax.legend(loc='upper right', markerscale=4, fontsize=9)


class MeasurementSession:
    """Interactive click-to-measure session — shows one scan at a time."""

    def __init__(self, fig, ax, captures, colors, output_dir='figures/map_evaluation'):
        self.fig = fig
        self.ax = ax
        self.captures = captures
        self.colors = colors
        self.output_dir = output_dir
        self.results = {}
        self.wp_idx = 0
        self.feat_idx = 0
        self.pending = None
        self._tmp_artists = []
        self._scan_artists = []   # current wp scan scatter + robot marker

        self._cid_click = fig.canvas.mpl_connect('button_press_event', self._on_click)
        self._cid_key   = fig.canvas.mpl_connect('key_press_event',   self._on_key)

        self._draw_current_scan()
        self._update_prompt()

    def _wp(self):
        return self.captures[self.wp_idx]

    def _feature(self):
        return FEATURES[self.feat_idx]

    def _robot_pos(self):
        _, meta, _, _ = self._wp()
        return meta['pose']['x'], meta['pose']['y']

    def _clear_scan(self):
        for a in self._scan_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._scan_artists = []

    def _clear_tmp(self):
        for a in self._tmp_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._tmp_artists = []

    def _draw_current_scan(self):
        self._clear_scan()
        wp_id, meta, gx, gy = self._wp()
        color = self.colors[self.wp_idx]

        # Dim all other scans in grey
        for i, (other_id, other_meta, ogx, ogy) in enumerate(self.captures):
            if i != self.wp_idx:
                sc = self.ax.scatter(ogx, ogy, s=DOT_SIZE_DIM, color='0.75', alpha=0.3, zorder=1)
                self._scan_artists.append(sc)

        # Current scan bright
        sc = self.ax.scatter(gx, gy, s=DOT_SIZE, color=color, alpha=0.9, zorder=2,
                             label=f'WP {wp_id}')
        self._scan_artists.append(sc)

        rx, ry = meta['pose']['x'], meta['pose']['y']
        yaw = meta['pose']['yaw_rad']
        m, = self.ax.plot(rx, ry, 'o', color=color, markersize=10,
                          markeredgecolor='black', linewidth=0.5, zorder=6)
        self._scan_artists.append(m)
        arrow_len = 0.2
        ann = self.ax.annotate('',
                    xy=(rx + arrow_len * math.cos(yaw), ry + arrow_len * math.sin(yaw)),
                    xytext=(rx, ry),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2))
        self._scan_artists.append(ann)

        self.fig.canvas.draw_idle()

    def _update_prompt(self):
        wp_id, _, _, _ = self._wp()
        self.fig.suptitle(
            f'WP {wp_id}  |  Click on: {self._feature()}\n'
            f'[click] mark point    [Enter / Space] confirm & next    [r] redo',
            fontsize=11
        )
        self.fig.canvas.draw_idle()

    def _on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        if self.wp_idx >= len(self.captures):
            return

        cx, cy = event.xdata, event.ydata
        rx, ry = self._robot_pos()
        dist = math.sqrt((cx - rx) ** 2 + (cy - ry) ** 2)
        self.pending = (cx, cy, dist)

        self._clear_tmp()
        self._tmp_artists += self.ax.plot(cx, cy, 'r*', markersize=16, zorder=10)
        self._tmp_artists += self.ax.plot([rx, cx], [ry, cy], 'r--', lw=1.5, zorder=9)

        wp_id, _, _, _ = self._wp()
        self.fig.suptitle(
            f'WP {wp_id}  |  {self._feature()}: {dist:.3f} m\n'
            f'[click] new point    [Enter / Space] confirm & next    [r] redo',
            fontsize=11
        )
        self.fig.canvas.draw_idle()

    def _on_key(self, event):
        if event.key in ('enter', ' '):
            self._confirm()
        elif event.key == 'r':
            self._redo()

    def _confirm(self):
        if self.pending is None:
            return
        cx, cy, dist = self.pending
        rx, ry = self._robot_pos()
        wp_id, _, _, _ = self._wp()

        self.results.setdefault(wp_id, {})[self._feature()] = dist

        self._clear_tmp()
        self.ax.plot(cx, cy, 'g*', markersize=16, zorder=10)
        self.ax.plot([rx, cx], [ry, cy], 'g--', lw=1.5, zorder=9)
        self.ax.annotate(f'{dist:.3f} m', (cx, cy), textcoords='offset points',
                         xytext=(6, 6), fontsize=8, color='green')
        self.pending = None

        self.feat_idx += 1
        if self.feat_idx >= len(FEATURES):
            self._save_wp_figure(wp_id)
            self.feat_idx = 0
            self.wp_idx += 1
            if self.wp_idx >= len(self.captures):
                self._finish()
                return
            self._draw_current_scan()

        self._update_prompt()
        self.fig.canvas.draw_idle()

    def _save_wp_figure(self, wp_id):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, f'wp{wp_id}_measurements.png')
        self.fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'Saved {path}')

    def _redo(self):
        self._clear_tmp()
        self.pending = None
        self._update_prompt()
        self.fig.canvas.draw_idle()

    def _finish(self):
        self._clear_scan()
        # Show full overlay on finish
        for color, (wp_id, meta, gx, gy) in zip(self.colors, self.captures):
            self.ax.scatter(gx, gy, s=DOT_SIZE, color=color, alpha=0.7)
            draw_robot(self.ax, meta, color)
        self.fig.canvas.mpl_disconnect(self._cid_click)
        self.fig.canvas.mpl_disconnect(self._cid_key)
        self.fig.suptitle('All measurements complete — see terminal for results.', fontsize=12)
        self.fig.canvas.draw_idle()
        self._print_results()

    def _print_results(self):
        print('\n=== Matplotlib Measurement Results ===')
        header = f'{"WP":<4}  {"North Wall (m)":<16}  {"East Door (m)":<15}  {"Bin Corner (m)":<14}'
        print(header)
        print('-' * len(header))
        for wp_id in sorted(self.results):
            r = self.results[wp_id]
            nw = f"{r['North Wall']:.3f}"  if 'North Wall'  in r else '—'
            ed = f"{r['East Door']:.3f}"   if 'East Door'   in r else '—'
            bc = f"{r['Bin Corner']:.3f}"  if 'Bin Corner'  in r else '—'
            print(f'{wp_id:<4}  {nw:<16}  {ed:<15}  {bc:<14}')
        print()


# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Overlay all waypoint scan captures')
    parser.add_argument('--captures-dir', default='data/captures',
                        help='Path to captures directory (default: data/captures)')
    parser.add_argument('--output', default=None,
                        help='Save figure to this path instead of displaying')
    parser.add_argument('--measure', action='store_true',
                        help='Enable interactive click-to-measure mode')
    args = parser.parse_args()

    yaml_files = sorted(glob.glob(os.path.join(args.captures_dir, 'wp_*.yaml')))
    if not yaml_files:
        print(f'No captures found in {args.captures_dir}')
        return

    by_wp = {}
    for yf in yaml_files:
        with open(yf) as f:
            meta = yaml.safe_load(f)
        wp_id = meta['waypoint_id']
        if wp_id not in by_wp or yf > by_wp[wp_id]:
            by_wp[wp_id] = yf

    waypoints = sorted(by_wp.keys())
    colors = cm.tab10(np.linspace(0, 1, len(waypoints)))

    captures = []
    for wp_id in waypoints:
        meta, ranges = load_capture(by_wp[wp_id])
        gx, gy = scan_to_global_points(meta, ranges)
        captures.append((wp_id, meta, gx, gy))

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.3)

    if args.measure:
        ax.set_title('Overlaid Waypoint Scans (odom frame)', fontsize=14)
        # Fix axis limits up front so zoom doesn't change when scans swap
        all_gx = np.concatenate([gx for _, _, gx, _ in captures])
        all_gy = np.concatenate([gy for _, _, _, gy in captures])
        pad = 0.5
        ax.set_xlim(all_gx.min() - pad, all_gx.max() + pad)
        ax.set_ylim(all_gy.min() - pad, all_gy.max() + pad)
        plt.tight_layout()
        session = MeasurementSession(fig, ax, captures, colors)
        plt.show()
    elif args.output:
        ax.set_title('Overlaid Waypoint Scans (odom frame)', fontsize=14)
        build_overlay(ax, captures, colors)
        plt.tight_layout()
        os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
        plt.savefig(args.output, dpi=150)
        print(f'Saved to {args.output}')
    else:
        ax.set_title('Overlaid Waypoint Scans (odom frame)', fontsize=14)
        build_overlay(ax, captures, colors)
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
