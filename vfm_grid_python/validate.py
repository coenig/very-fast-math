"""Independent numerical checks on concrete endpoint motion and milestones."""
from .common import states


def validate_trace(graph,geometry,trace):
    frames=states(trace)
    cars=len(graph["milestones"][0])
    sections=geometry["sections"]
    connections={(c["from"],c["to"]):c for c in geometry["connections"]}

    def decode(frame,car):
        p=f"env.veh___6{car}9___"
        section=int(frame[p+".on_straight_section"])
        seclet=("s",section) if section>=0 else ("c",int(frame[p+".traversion_from"]),int(frame[p+".traversion_to"]))
        return seclet,int(frame[p+".abs_pos"]),int(frame[p+".v"]),int(frame[p+".a"])

    def length(seclet):
        return sections[seclet[1]]["length"] if seclet[0]=="s" else connections[seclet[1:]]["length"]

    def observed(seclet,pos,vel,node):
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
                if not observed(seclet,pos,vel,graph["milestones"][phase][car]):
                    raise ValueError(f"tick {index}: incorrect milestone observation")
            u=graph["milestones"][phase][car]
            v=graph["milestones"][min(phase+1,len(graph["milestones"])-1)][car]
            travelling=(seclet[0]=="s" and u!=v and set(sections[seclet[1]]["edge"])=={u,v})
            if not (travelling or observed(seclet,pos,vel,u) or observed(seclet,pos,vel,v)):
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
            resources.append(occupied)
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
                if resources[car] & resources[other]:
                    raise ValueError(f"tick {index}: conflicting occupancy")
                if index and (resources[car] | previous_resources[car]) & (resources[other] | previous_resources[other]):
                    raise ValueError(f"tick {index}: conflicting swept occupancy")
        previous_resources=resources
    if int(frames[-1]["refinement_phase"]) != len(graph["milestones"])-1:
        raise ValueError("trace did not complete refinement")
    return {"valid":True,"states":len(frames),"cars":cars,"milestones":len(graph["milestones"])}
