"""
plot_path.py -- Parse and plot a single test output from path.txt

Usage:
    python plot_path.py              (reads path.txt)
    python plot_path.py myfile.txt   (reads myfile.txt)

Parses envelope, forbidden zones, start, target, and step data,
then plots the result with matplotlib.
"""

import re
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random


def parse_file(filename):
    with open(filename, 'r') as f:
        text = f.read()

    # --- Test name ---
    m = re.search(r'^\s{2}(\S.*\S)\s*$', text, re.MULTILINE)
    test_name = m.group(1) if m else 'Unknown'
    # Skip lines that are just '=' separators
    if re.match(r'^=+$', test_name):
        test_name = 'Test'

    # Try to find a better name (line after ====)
    m = re.search(r'={5,}\n\s{2}([^=\n]+?)\s*\n={5,}', text)
    if m:
        test_name = m.group(1).strip()

    # --- Envelope ---
    m = re.search(r'Envelope:\s*AZ\[([^,]+),([^\]]+)\]\s*EL\[([^,]+),([^\]]+)\]', text)
    if m:
        envelope = (float(m.group(1)), float(m.group(2)),
                    float(m.group(3)), float(m.group(4)))
    else:
        envelope = (-180, 180, -90, 90)

    # --- az_wrap ---
    m = re.search(r'az_wrap=(\d)', text)
    az_wrap = int(m.group(1)) if m else 0

    # --- Forbidden zones ---
    forbidden = []
    for m in re.finditer(
        r'\[\s*\d+\]\s*AZ\[\s*([^,]+),\s*([^\]]+)\]\s*EL\[\s*([^,]+),\s*([^\]]+)\]',
        text
    ):
        forbidden.append((
            float(m.group(1)), float(m.group(2)),
            float(m.group(3)), float(m.group(4))
        ))

    # --- Start and Target ---
    m = re.search(r'Start:\s*\(([^,]+),\s*([^)]+)\)\s*Target:\s*\(([^,]+),\s*([^)]+)\)', text)
    if m:
        start = (float(m.group(1)), float(m.group(2)))
        target = (float(m.group(3)), float(m.group(4)))
    else:
        start = (0, 0)
        target = (0, 0)

    # --- Step data ---
    steps = []
    # With cmd
    for m in re.finditer(
        r'\[\s*(\d+)\]\s*pos=\(\s*([^,]+),\s*([^)]+)\)\s*'
        r'cmd=\(\s*([^,]+),\s*([^)]+)\)\s*'
        r'->\s*next=\(\s*([^,]+),\s*([^)]+)\)\s*(\S+)',
        text
    ):
        steps.append({
            'step': int(m.group(1)),
            'pos_az': float(m.group(2)), 'pos_el': float(m.group(3)),
            'cmd_az': float(m.group(4)), 'cmd_el': float(m.group(5)),
            'next_az': float(m.group(6)), 'next_el': float(m.group(7)),
            'status': m.group(8).strip(),
        })
    # Without cmd (older format)
    if not steps:
        for m in re.finditer(
            r'\[\s*(\d+)\]\s*pos=\(\s*([^,]+),\s*([^)]+)\)\s*'
            r'->\s*next=\(\s*([^,]+),\s*([^)]+)\)\s*(\S+)',
            text
        ):
            steps.append({
                'step': int(m.group(1)),
                'pos_az': float(m.group(2)), 'pos_el': float(m.group(3)),
                'cmd_az': target[0], 'cmd_el': target[1],
                'next_az': float(m.group(4)), 'next_el': float(m.group(5)),
                'status': m.group(6).strip(),
            })

    # --- PASS / FAIL ---
    m = re.search(r'RESULT:\s*(PASS|FAIL)', text)
    result = m.group(1) if m else 'N/A'

    return {
        'name': test_name,
        'envelope': envelope,
        'az_wrap': az_wrap,
        'forbidden': forbidden,
        'start': start,
        'target': target,
        'steps': steps,
        'result': result,
    }


def plot(data):
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))

    env = data['envelope']
    forbidden = data['forbidden']
    steps = data['steps']
    start = data['start']
    target = data['target']

    # --- Forbidden zones (draw first) ---
    for i, fz in enumerate(forbidden):
        az_min, az_max, el_min, el_max = fz
        rect = plt.Rectangle(
            (az_min, el_min), az_max - az_min, el_max - el_min,
            linewidth=1, edgecolor='red', facecolor='red',
            alpha=0.25, zorder=1
        )
        ax.add_patch(rect)
        rect_h = plt.Rectangle(
            (az_min, el_min), az_max - az_min, el_max - el_min,
            linewidth=0.5, edgecolor='red', facecolor='none',
            hatch='///', zorder=1
        )
        ax.add_patch(rect_h)
        # Label
        cx = (az_min + az_max) / 2
        cy = (el_min + el_max) / 2
        ax.text(cx, cy, f'F{i}', ha='center', va='center',
                fontsize=8, color='red', fontweight='bold', zorder=2)

    # --- Envelope border (on top) ---
    env_rect = plt.Rectangle(
        (env[0], env[2]), env[1] - env[0], env[3] - env[2],
        linewidth=2, edgecolor='black', facecolor='none',
        linestyle='--', zorder=8
    )
    ax.add_patch(env_rect)

    # --- Trajectory ---
    if steps:
        pos_az = [s['pos_az'] for s in steps]
        pos_el = [s['pos_el'] for s in steps]

        # Handle wrap discontinuity at +/-180
        if data['az_wrap']:
            segments_az, segments_el = [], []
            seg_az, seg_el = [pos_az[0]], [pos_el[0]]
            for i in range(1, len(pos_az)):
                if abs(pos_az[i] - pos_az[i - 1]) > 180:
                    segments_az.append(seg_az)
                    segments_el.append(seg_el)
                    seg_az, seg_el = [], []
                seg_az.append(pos_az[i])
                seg_el.append(pos_el[i])
            segments_az.append(seg_az)
            segments_el.append(seg_el)
            for sa, se in zip(segments_az, segments_el):
                ax.plot(sa, se, '-', color='#2196F3', linewidth=1.5, zorder=3)
        else:
            ax.plot(pos_az, pos_el, '-', color='#2196F3', linewidth=1.5, zorder=3)

        # Direction arrows
        n = max(1, len(pos_az) // 12)
        for i in range(0, len(pos_az) - 1, n):
            daz = pos_az[i + 1] - pos_az[i]
            d_el = pos_el[i + 1] - pos_el[i]
            if data['az_wrap'] and abs(daz) > 180:
                continue
            if abs(daz) > 0.01 or abs(d_el) > 0.01:
                ax.annotate('',
                    xy=(pos_az[i] + daz * 0.6, pos_el[i] + d_el * 0.6),
                    xytext=(pos_az[i], pos_el[i]),
                    arrowprops=dict(arrowstyle='->', color='#2196F3', lw=1.2),
                    zorder=4)

        # Waypoint command dots (unique)
        wp_set = set()
        for s in steps:
            if s['status'] == 'OK_WAYPOINT':
                wp_set.add((round(s['next_az'], 1), round(s['next_el'], 1)))
        if wp_set:
            wp_az = [w[0] for w in wp_set]
            wp_el = [w[1] for w in wp_set]
            ax.plot(wp_az, wp_el, 'o', color='#FF9800', markersize=7,
                    zorder=5, label='Waypoint cmd')

    # --- Start and Target markers ---
    ax.plot(start[0], start[1], 'o', color='#4CAF50', markersize=10,
            markeredgecolor='black', markeredgewidth=0.5, zorder=6,
            label=f'Start ({start[0]:.1f}, {start[1]:.1f})')
    ax.plot(target[0], target[1], '*', color='#F44336', markersize=14,
            markeredgecolor='black', markeredgewidth=0.5, zorder=6,
            label=f'Target ({target[0]:.1f}, {target[1]:.1f})')

    # --- Axis limits ---
    pad_az = (env[1] - env[0]) * 0.05
    pad_el = (env[3] - env[2]) * 0.05
    ax.set_xlim(env[0] - pad_az, env[1] + pad_az)
    ax.set_ylim(env[2] - pad_el, env[3] + pad_el)

    ax.set_xlabel('AZ (deg)', fontsize=11)
    ax.set_ylabel('EL (deg)', fontsize=11)
    ax.grid(True, alpha=0.3, linewidth=0.5)

    # --- Title ---
    color = '#4CAF50' if data['result'] == 'PASS' else '#F44336'
    n_steps = len(steps)
    ax.set_title(f"{data['name']}  [{data['result']}]  ({n_steps} steps)",
                 fontsize=13, fontweight='bold', color=color)

    # --- Legend ---
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#4CAF50',
                   markersize=8, markeredgecolor='black', label='Start'),
        plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='#F44336',
                   markersize=12, markeredgecolor='black', label='Target'),
        plt.Line2D([0], [0], color='#2196F3', linewidth=1.5, label='Path'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#FF9800',
                   markersize=6, label='Waypoint cmd'),
        mpatches.Patch(facecolor='red', alpha=0.25, edgecolor='red',
                       label='Forbidden zone'),
        plt.Line2D([0], [0], color='black', linewidth=2, linestyle='--',
                   label='Envelope'),
    ]
    ax.legend(handles=legend_elements, loc='best', fontsize=9,
              frameon=True, fancybox=True)

    # --- Info box ---
    info = f"Envelope: AZ[{env[0]:.0f},{env[1]:.0f}] EL[{env[2]:.0f},{env[3]:.0f}]"
    info += f"  az_wrap={data['az_wrap']}"
    info += f"\nForbidden zones: {len(forbidden)}"
    for i, fz in enumerate(forbidden):
        info += f"\n  F{i}: AZ[{fz[0]:.1f},{fz[1]:.1f}] EL[{fz[2]:.1f},{fz[3]:.1f}]"
    ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=7,
            verticalalignment='bottom', fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='wheat', alpha=0.7),
            zorder=10)
    #plt.show()
    plt.tight_layout()
    a = random.randint(0, 99999999999)
    plt.savefig(f'figure\path_plot_{a}.png', dpi=150, bbox_inches='tight')
    print(f"Saved path_plot_{a}.png")
    


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else 'forbidden_zones_c.txt'
    print(f"Reading {filename}...")
    data = parse_file(filename)

    print(f"Test:     {data['name']}")
    print(f"Envelope: AZ[{data['envelope'][0]:.1f}, {data['envelope'][1]:.1f}]"
          f"  EL[{data['envelope'][2]:.1f}, {data['envelope'][3]:.1f}]"
          f"  az_wrap={data['az_wrap']}")
    print(f"Forbidden zones: {len(data['forbidden'])}")
    for i, fz in enumerate(data['forbidden']):
        print(f"  [{i:2d}] AZ[{fz[0]:7.1f}, {fz[1]:7.1f}]  EL[{fz[2]:6.1f}, {fz[3]:6.1f}]")
    print(f"Start:  ({data['start'][0]:.2f}, {data['start'][1]:.2f})")
    print(f"Target: ({data['target'][0]:.2f}, {data['target'][1]:.2f})")
    print(f"Steps:  {len(data['steps'])}")
    print(f"Result: {data['result']}")

    plot(data)


if __name__ == '__main__':
    main()
