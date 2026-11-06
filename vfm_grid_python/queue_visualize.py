"""Draw an ordered-queue witness, without pretending to have concrete geometry.

Only enabled roads are shown. Dot coordinates are representative placements
respecting queue order and endpoint flags, NOT reconstructed vehicle dynamics.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from vfm_grid_python.common import states


def draw(directory, gif=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    directory = Path(directory)
    config = json.loads((directory/'config.json').read_text())
    inv = json.loads((directory/'inventory.json').read_text())
    trace = json.loads((directory/'queue_trace.json').read_text())
    first = states(directory/'trace.xml')[0]
    joints = inv['joints']
    def active(road):
        return all(first['edge_%s_%s' % tuple(sorted((joints[j]['node'], joints[j]['neighbour'])))] == 'TRUE'
                   for j in road['joints'])
    roads = [r for r in inv['roads'] if active(r)]
    fig, ax = plt.subplots(figsize=(9, max(3, len(roads)*.38+1)))
    colors = plt.get_cmap('tab10')
    def update(t):
        ax.clear()
        labels = []
        for y, road in enumerate(roads):
            r = road['id']
            section = road['kind'] == 'section'
            length = config.get('connection_length', 12)
            if section:
                length = config.get('parking_length', 7) if any(first[f'parking_{u}'] == 'TRUE' for u in road['edge']) else config.get('section_length', 20)
            ax.plot([0, length], [y,y], color='black' if section else 'royalblue', lw=2)
            occupants = sorted((c | {'car': i} for i,c in enumerate(trace[t]) if c['road'] == r), key=lambda c:c['rank'])
            n = len(occupants)
            left = 0 if any(c['tip'] == 'left' for c in occupants) else 1
            right = length if any(c['tip'] == 'right' for c in occupants) else length-1
            for k,c in enumerate(occupants):
                x = 0 if c['tip'] == 'left' else length if c['tip'] == 'right' else (left+right)/2 if n == 1 else left+k*(right-left)/(n-1)
                ax.scatter(x, y, color=colors(c['car'] % 10), s=170, zorder=3)
                ax.text(x, y, str(c['car']), ha='center', va='center', color='white', fontsize=8, zorder=4)
            a,b = road['joints']
            labels.append(f"{'S' if section else 'C'}{r}: j{a}(N{joints[a]['node']})—j{b}(N{joints[b]['node']})")
        ax.set_yticks(range(len(roads)), labels)
        ax.invert_yaxis()
        ax.set_xlabel('Representative road coordinate (not a concrete trajectory)')
        ax.set_title(f'Queue witness — state {t}/{len(trace)-1}\nBlack: section; blue: connection; numbered dots: cars')
        fig.tight_layout()
    update(len(trace)-1)
    fig.savefig(directory/'queues_final.png', dpi=140)
    if gif:
        animation = FuncAnimation(fig, update, frames=len(trace), interval=500)
        animation.save(directory/'queues.gif', writer=PillowWriter(fps=2))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    draw(args.directory, not args.no_gif)


if __name__ == '__main__':
    main()
