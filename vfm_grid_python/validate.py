"""Independent numerical checks on concrete endpoint motion and milestones."""
from .common import states


def validate_trace(graph,geometry,trace):
    frames=states(trace)
    cars=len(graph["milestones"][0])
    sections=geometry["sections"]
    connections={(c["from"],c["to"]):c for c in geometry["connections"]}
    stable=graph.get('semantics')=='stable_concurrent'
    shared=bool(graph.get('junction_extension'))
    native=False
    if shared:
        from .junctions import conflicts, route, section_for, validate_witness, native_collisions, holding_cars
        validate_witness(graph,geometry)
        native=native_collisions(graph)
        conflicting=set() if native else {frozenset(pair) for pair in conflicts(geometry)}
    if stable:
        from .stable import anchors
        choices=anchors(graph)
        holding={u:next(s for s in sections if set(s['edge'])=={u,v})
                 for u,v in choices.items() if v>=0}

    def decode(frame,car):
        p=f"env.veh___6{car}9___"
        section=int(frame[p+".on_straight_section"])
        seclet=("s",section) if section>=0 else ("c",int(frame[p+".traversion_from"]),int(frame[p+".traversion_to"]))
        return seclet,int(frame[p+".abs_pos"]),int(frame[p+".v"]),int(frame[p+".a"])

    def length(seclet):
        return sections[seclet[1]]["length"] if seclet[0]=="s" else connections[seclet[1:]]["length"]

    def observed(seclet,pos,vel,node,acc=0):
        if stable:
            s=holding[node]
            endpoint=s['length'] if s['edge'][1]==node else 0
            return seclet==('s',s['id']) and pos==endpoint and vel==0 and acc==0
        kind=next(n["kind"] for n in geometry["nodes"] if n["id"]==node)
        if seclet[0]=="c":
            return kind=="N" and connections[seclet[1:]]["node"]==node
        s=sections[seclet[1]]
        match=(s["edge"][0]==node and pos==0) or (s["edge"][1]==node and pos==s["length"])
        return match and (kind!="P" or vel==0)

    for index,frame in enumerate(frames):
        phase=int(frame["refinement_phase"])
        if index and phase not in (int(frames[index-1]["refinement_phase"]),int(frames[index-1]["refinement_phase"])+1):
            raise ValueError("monitor skipped a milestone")
        resources=[]
        for car in range(cars):
            seclet,pos,vel,acc=decode(frame,car)
            if not 0<=pos<=length(seclet):
                raise ValueError(f"tick {index}, car {car}: position outside seclet")
            if index==0 or phase!=int(frames[index-1]["refinement_phase"]):
                if graph.get('section_only_milestones', False) and seclet[0]!='s':
                    raise ValueError(f"tick {index}: milestone inside a connection")
                if not observed(seclet,pos,vel,graph["milestones"][phase][car],acc):
                    raise ValueError(f"tick {index}: incorrect milestone observation")
                if shared:
                    expected=section_for(geometry,graph['milestones'][phase][car],graph['approaches'][phase][car])
                    if seclet!=('s',expected['id']):
                        raise ValueError(f'tick {index}: incorrect milestone approach')
            u=graph["milestones"][phase][car]
            v=graph["milestones"][min(phase+1,len(graph["milestones"])-1)][car]
            travelling=(seclet[0]=="s" and u!=v and set(sections[seclet[1]]["edge"])=={u,v})
            permitted=travelling or observed(seclet,pos,vel,u,acc) or observed(seclet,pos,vel,v,acc)
            if shared:
                arrival=graph['approaches'][min(phase+1,len(graph['milestones'])-1)][car]
                permitted=seclet in route(geometry,u,graph['approaches'][phase][car],v,arrival)
                if graph.get('stationary_holds'):
                    if index == 0 and (vel != 0 or acc != 0):
                        raise ValueError(f'tick 0, car {car}: initial state is not at rest')
                    if car in holding_cars(graph,phase):
                        anchor=section_for(geometry,u,graph['approaches'][phase][car])
                        endpoint=anchor['length'] if anchor['edge'][1]==u else 0
                        if (seclet,pos,vel,acc) != (('s',anchor['id']),endpoint,0,0):
                            raise ValueError(f'tick {index}, car {car}: abstract hold must remain at rest at its attachment')
            if stable:
                if u==v:
                    permitted=observed(seclet,pos,vel,u,acc)
                else:
                    travel=next(s for s in sections if set(s['edge'])=={u,v})
                    permitted=seclet==('s',travel['id'])
                    for node in (u,v):
                        anchor=holding[node]
                        endpoint=anchor['length'] if anchor['edge'][1]==node else 0
                        permitted |= seclet==('s',anchor['id']) and abs(pos-endpoint)<=1
                        if seclet[0]=='c':
                            c=connections[seclet[1:]]
                            permitted |= c['node']==node and {c['from'],c['to']}=={anchor['id'],travel['id']}
            if not permitted:
                raise ValueError(f"tick {index}: car deviated from abstract route")
            occupied=set()
            if seclet[0]=="c":
                occupied.add(("junction",connections[seclet[1:]]["node"]))
            else:
                s=sections[seclet[1]]
                occupied.add(("edge",s["id"]))
                vehicle_length=int(frame.get("env.veh_length",5))
                for node,distance in zip(s["edge"],(pos,s["length"]-pos)):
                    kind=next(n["kind"] for n in geometry["nodes"] if n["id"]==node)
                    if kind=="N" and distance<=vehicle_length:
                        occupied.add(("junction",node))
            resources.append({seclet} if shared else occupied)
            if index==0:
                continue
            old,op,ov,oa=decode(frames[index-1],car)
            limit=int(frame.get("env.max_vel", "1000000"))
            step=max(-limit,min(limit,ov+oa))
            raw=op+step
            expected_pos,expected_vel=raw,step
            if old!=seclet:
                if old[0]=="s" and seclet[0]=="c":
                    c=connections[seclet[1:]]
                    if old[1]==c["from"]:
                        expected_pos=raw-length(old) if c["from_end"] else -raw
                        expected_vel=step if c["from_end"] else -step
                    elif old[1]==c["to"]:
                        expected_pos=c["length"]-(raw-length(old)) if c["to_end"] else c["length"]+raw
                        expected_vel=-step if c["to_end"] else step
                    else: raise ValueError("unconnected section entry")
                elif old[0]=="c" and seclet[0]=="s":
                    c=connections[old[1:]]
                    if seclet[1]==c["from"] and raw<0:
                        expected_pos=length(seclet)+raw if c["from_end"] else -raw
                        expected_vel=step if c["from_end"] else -step
                    elif seclet[1]==c["to"] and raw>c["length"]:
                        expected_pos=length(seclet)-(raw-c["length"]) if c["to_end"] else raw-c["length"]
                        expected_vel=-step if c["to_end"] else step
                    else: raise ValueError("invalid connector exit")
                else: raise ValueError("skipped connector/section")
            if (pos,vel)!=(expected_pos,expected_vel):
                raise ValueError(f"tick {index}, car {car}: inconsistent displacement/speed")
        for car in range(cars):
            for other in range(car):
                if native:
                    x,xp,_,_=decode(frame,car)
                    y,yp,_,_=decode(frame,other)
                    if x==y and abs(xp-yp)<=int(frame.get('env.veh_length',5)):
                        raise ValueError(f'tick {index}: native same-seclet separation violated')
                    if index:
                        ox,op,_,_=decode(frames[index-1],car)
                        oy,oq,_,_=decode(frames[index-1],other)
                        if ox==oy==x==y and (op<oq)!=(xp<yp):
                            raise ValueError(f'tick {index}: native same-seclet order reversed')
                    continue
                def intersects(a,b):
                    return (any(frozenset((x,y)) in conflicting for x in a for y in b)
                            if shared else bool(a & b))
                if intersects(resources[car], resources[other]):
                    raise ValueError(f"tick {index}: conflicting occupancy")
                if index and intersects(resources[car] | previous_resources[car], resources[other] | previous_resources[other]):
                    raise ValueError(f"tick {index}: conflicting swept occupancy")
        previous_resources=resources
    if int(frames[-1]["refinement_phase"]) != len(graph["milestones"])-1:
        raise ValueError("trace did not complete refinement")
    report={"valid":True,"states":len(frames),"cars":cars,"milestones":len(graph["milestones"])}
    if shared:
        report['stationary_holds']=bool(graph.get('stationary_holds'))
    if native:
        report.update(collision_policy='native same-seclet separation/order',
                      cross_element_and_between_tick_geometry_checked=False,
                      greedy_dynamic_completeness_claimed=False)
    return report
