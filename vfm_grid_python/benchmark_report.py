"""Small local comparison reports; timeouts are not successful witnesses."""
import argparse
import json
from pathlib import Path

def write_report(root, rows):
    root=Path(root)
    lines=['# Queue-compatible approach comparison','',
           'Checking excludes generation, preprocessing, validation, export and GIF rendering.',
           'Enumeration search time adds candidate enumeration to concrete checking.',
           'A timeout or bounded failure is not an unsatisfiability proof.','',
           '| Instance | Approach | Status | Abstract check s | Concrete check s | Enumeration s | Wall s |',
           '|---|---|---|---:|---:|---:|---:|']
    def value(x): return '—' if x is None else f'{x:.3f}'
    for r in rows:
        a=r.get('abstract_timings_seconds',{}).get('model_checking',0)
        c=r.get('concrete',{}).get('timings_seconds',r.get('timings_seconds',{})).get('model_checking')
        lines.append(f"| {r['instance']} | {r['approach']} | {r['status']} | {value(a)} | {value(c)} | {value(r.get('enumeration_seconds'))} | {value(r.get('wall_seconds'))} |")
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    methods=['external_enumeration','direct_model_checking','abstraction_based','external_enumeration_sat']
    table=['# Search times (seconds)','', 'Only validated witnesses have numeric cells; other cells show status.','',
           '| Instance | External enumeration + checking | Direct checking | Abstract + concrete checking | gamma-SAT enumeration + checking |',
           '|---|---:|---:|---:|---:|']
    for name in dict.fromkeys(r['instance'] for r in rows):
        group={r['approach']:r for r in rows if r['instance']==name}
        cells=[]
        for method in methods:
            r=group.get(method)
            if not r: cells.append('—'); continue
            if r['status']!='witness':cells.append(r['status']);continue
            metric=r['model_checking_seconds']+(r.get('enumeration_seconds',0) if method=='external_enumeration' else 0)
            cells.append(value(metric))
        table.append('| '+name+' | '+' | '.join(cells)+' |')
    (root/'COMPARISON.md').write_text('\n'.join(table)+'\n')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    a=p.parse_args()
    write_report(a.directory,json.loads((a.directory/'results.json').read_text()))

if __name__=='__main__':main()
