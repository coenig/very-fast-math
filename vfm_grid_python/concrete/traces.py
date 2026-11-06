"""Complete, model-validated XML/text export (independent of refinement strategy)."""
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from vfm_grid_python.common import ROOT, dump, execute, states
from .schema import OCCUPANCY, flatten

def geometry_values(geometry):
    """Native-renderer metadata in global coordinates, no symbolic full model."""
    result={}
    outgoing={s['id']:[] for s in geometry['sections']}
    def put(name,value): result['env.'+name]=str(value)
    for s in geometry['sections']:
        i=s['id']; angle=s['angle']
        put(f'section_{i}_end',s['length'])
        for point in ('source','drain'):
            for axis,value in zip(('x','y'),s[point]): put(f'section_{i}.{point}.{axis}',value)
        put(f'section_{i}.angle',angle); put(f'section_{i}.angle_raw',angle//45)
        put(f'sin_of_section_{i}_angle',round(100*math.sin(math.radians(angle))))
        put(f'cos_of_section_{i}_angle',round(100*math.cos(math.radians(angle))))
        put(f'section_{i}_segment_0_pos_begin',0)
        put(f'section_{i}_segment_0_min_lane',0); put(f'section_{i}_segment_0_max_lane',0)
    for c in geometry['connections']:
        a,b=c['from'],c['to']; outgoing[a].append(b)
        put(f'endpoint_{a}_{b}_from_end',str(c['from_end']).upper())
        put(f'endpoint_{a}_{b}_to_end',str(c['to_end']).upper())
        put(f'arclength_from_sec_{a}_to_sec_{b}_on_lane_0',c['length'])
    slots=max(1,max(map(len,outgoing.values())))
    for section,targets in outgoing.items():
        for slot,target in enumerate(targets+[-1]*(slots-len(targets))):
            put(f'outgoing_connection_{slot}_of_section_{section}',target)
    return result


def export_trace(frames,geometry,directory,complete,model=None,description=None,validate_geometry=True):
    """Full XML states and delta-encoded text, in nuXmv's trace format.

Only a model-completed, validated export is marked complete. Partial files
remain useful for diagnostics before the global model has been generated.
"""
    prefix='' if complete else 'partial_'
    if complete and model is None:
        raise ValueError('a complete export requires the global model inventory')
    if model is not None:
        from vfm_grid_python.benchmark_timing import interval
        with interval('validation'):
            frames=model.complete(frames,geometry,preserve_topology=not validate_geometry)
            model.validate(frames)
        static=geometry_values(geometry)
        for name,value in (static.items() if validate_geometry else ()):
            if name in frames[0] and frames[0][name]!=value:
                raise ValueError(f'global model geometry mismatch: {name}')
    description = description or ('Python-stitched sequential concrete trace' if complete else 'Partial sequential concrete trace')
    root=ET.Element('counter-example',{'type':'0','id':'1','desc':description})
    text=['Trace Description: '+description, 'Trace Type: Counterexample']
    # Clear aliases that occurred in a different patch, otherwise sparse trace
    # readers can retain an old TRUE traversal flag across a boundary.
    aliases={k for s in frames for k in s if OCCUPANCY.match(k)}
    static=geometry_values(geometry)
    old={}
    for i,frame in enumerate(frames):
        current=frame if model is not None else {**static,**{k:'0' for k in aliases},**frame}
        node=ET.SubElement(root,'node')
        state=ET.SubElement(node,'state',{'id':str(i+1)})
        text.append(f'  -> State: 1.{i+1} <-')
        for key,value in sorted(current.items()):
            ET.SubElement(state,'value',{'variable':key}).text=str(value)
            if old.get(key)!=str(value):
                text.append(f'    {key} = {value}')
        old=current
    ET.SubElement(root,'loops').text=' '
    xml=directory/(prefix+'trace.xml')
    pending=directory/(prefix+'trace.pending.xml')
    ET.indent(root)
    ET.ElementTree(root).write(pending,encoding='unicode')
    if model is not None:
        with interval('validation'):
            report=model.validate(states(pending))
        if hasattr(model,'source'):
            report['xml_import']=model.check_xml_import(pending,directory/'xml_import',model.timeout)
        report.update(complete=complete,source='Model-completed trace, checked against full concrete model')
        dump(directory/(prefix+'model_validation.json'),report)
    pending.replace(xml)
    (directory/(prefix+'debug_trace_array.txt')).write_text('\n'.join(text)+'\n')
    return xml


def global_inventory(graph,geometry,config,output,template_config,timeout,model=None):
    """Generate/flatten the global model once, without a global solver search."""
    from vfm_grid_python.concrete.trace_model import TraceModel
    output.mkdir(parents=True,exist_ok=True)
    if model is None:
        dump(output/'generation_job.json',{'graph':graph,'geometry':geometry,
             'template_config':str(Path(template_config).resolve()),'vfm':config.get('vfm',{}),
             'scenario':config if config.get('scenario_stages') else None,
             'braking_margin':config.get('braking_margin',0)})
        result=execute([sys.executable,'-m','vfm_grid_python.concrete.generation','--worker',str(output)],
                       ROOT,output/'generation.log',timeout)
        dump(output/'generation_result.json',result)
        if result['status']!='finished': raise RuntimeError(f'global inventory generation {result["status"]}')
        model=Path(json.loads((output/'model.json').read_text())['model'])
    text,variables,result=flatten(model,output/'flat',timeout)
    inventory=TraceModel(text)
    inventory.source=(output/'flat/baseline.smv').resolve()
    inventory.timeout=timeout
    expected_cars={f'env.veh___6{i}9___.on_straight_section' for i in range(len(graph['milestones'][0]))}
    import re
    actual_cars={n for n in set(variables)|inventory.state_defines
                 if re.fullmatch(r'env\.veh___6\d+9___\.on_straight_section',n)}
    if actual_cars!=expected_cars: raise ValueError('global validation model has a different fleet')
    symbols=set(variables)|inventory.state_defines
    required=set(geometry_values(geometry))|{'refinement_phase'}
    if required-symbols:
        raise ValueError(f'global validation model lacks required geometry/monitor symbols: {sorted(required-symbols)[:8]}')
    expected_sections={f'env.section_{s["id"]}_end' for s in geometry['sections']}
    actual_sections={n for n in symbols if re.fullmatch(r'env\.section_\d+_end',n)}
    if actual_sections!=expected_sections: raise ValueError('global validation model has different sections')
    dump(output/'inventory.json',{'model':str(Path(model).resolve()),'variables':variables,
         'state_defines':sorted(inventory.state_defines),
         'transition_defines':sorted(n for n in inventory.defines
                                    if not n.startswith('__expr') and n not in inventory.state_defines)})
    return inventory
