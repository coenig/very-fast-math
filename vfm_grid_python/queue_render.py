"""Render completed local campaign witnesses after all measured attempts."""
import argparse
import json
from pathlib import Path
from .common import dump

def render_completed(root, retry=False):
    root=Path(root)
    rows=json.loads((root/'results.json').read_text())
    records=[]; links=['# Witness GIFs','', 'Abstract positions are representative queue placements, not concrete dynamics.','']
    for r in rows:
        output=Path(r['directory'])
        targets=[]
        if r.get('abstract',{}).get('status')=='witness': targets.append(('abstract',output/'abstract'))
        if r.get('status')=='witness' and r.get('trace'):targets.append(('concrete',Path(r['trace']).parent))
        for kind,directory in targets:
            record=dict(instance=r['instance'],approach=r['approach'],kind=kind)
            try:
                if kind=='abstract':
                    from .queue_visualize import draw
                    draw(directory)
                    gif=directory/'queues.gif'
                else:
                    from .visualize import concrete_plot
                    concrete_plot(json.loads((directory/'geometry.json').read_text()),directory,Path(r['trace']),True)
                    gif=directory/'concrete.gif'
                record.update(status='rendered',gif=str(gif))
                links.append(f"- {r['instance']}/{r['approach']}: [{kind}]({gif.relative_to(root)})")
            except Exception as exc:
                record.update(status='error',error=str(exc))
                links.append(f"- {r['instance']}/{r['approach']}: {kind} render error: {exc}")
            records.append(record)
    dump(root/'rendering.json',records)
    (root/'GIFS.md').write_text('\n'.join(links)+'\n')
    return records

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();render_completed(args.directory)

if __name__=='__main__':main()
