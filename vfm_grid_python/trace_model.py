"""Complete and check a finite trace against nuXmv's flattened concrete model.

This is an evaluator, not a solver: all persistent state must be supplied.
Unsupported syntax and missing values fail closed. DEFINEs are recomputed in
the global model, never copied from a differently numbered local model.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from .common import execute, states, dump
from .local_model import blocks, statements, schema, handoff, validate_value


TOKEN = re.compile(r'\s*(<->|->|:=|!=|<=|>=|\.\.|[(){};,?:!&|=<>+*/%-]|'
                   r'\d+|(?:"[^"\n]*"|[A-Za-z_$][\w$#]*)(?:\.(?:"[^"\n]*"|[A-Za-z_$][\w$#]*))*)')
PRECEDENCE = {'<->': 1, '->': 2, '|': 3, 'xor': 3, 'xnor': 3, '&': 4,
              '=': 5, '!=': 5, '<': 5, '>': 5, '<=': 5, '>=': 5, 'in': 5,
              'union': 6, '+': 7, '-': 7, '*': 8, '/': 8, 'mod': 8, '%': 8}


def parse(source):
    tokens=[]; pos=0
    while pos < len(source.rstrip()):
        match=TOKEN.match(source,pos)
        if not match:
            raise ValueError(f'unsupported SMV syntax: {source[pos:pos+100]}')
        tokens.append(match.group(1)); pos=match.end()
    tokens.append('<EOF>'); index=0
    # write_flat_model can emit thousands of nested parentheses for the
    # all-pairs occupancy constraints even on a modest grid. Reserve enough
    # Python frames for parsing and later evaluating that actual nesting.
    depth=maximum=0
    for token in tokens:
        if token=='(': depth+=1; maximum=max(maximum,depth)
        elif token==')': depth-=1
    needed=8*maximum+2000
    if needed>sys.getrecursionlimit(): sys.setrecursionlimit(needed)

    def take(expected=None):
        nonlocal index
        token=tokens[index]; index+=1
        if expected is not None and token!=expected:
            raise ValueError(f'expected {expected}, got {token} in {source[:150]}')
        return token

    def expression(minimum=0):
        token=take()
        if token in ('!', '-', '+'):
            left=('unary',token,expression(9))
        elif token=='(':
            left=expression(); take(')')
        elif token=='case':
            branches=[]
            while tokens[index]!='esac':
                condition=expression(); take(':'); value=expression(); take(';')
                branches.append((condition,value))
            take('esac'); left=('case',tuple(branches))
        elif token=='{':
            values=[]
            if tokens[index]!='}':
                values.append(expression())
                while tokens[index]==',':
                    take(','); values.append(expression())
            take('}'); left=('set',tuple(values))
        elif token.isdigit(): left=('literal',int(token))
        elif token in ('TRUE','FALSE'): left=('literal',token=='TRUE')
        elif re.fullmatch(TOKEN,token) and token not in PRECEDENCE and token not in (':',';',')','<EOF>'):
            if tokens[index]=='(':
                take('('); args=[expression()]
                while tokens[index]==',': take(','); args.append(expression())
                take(')')
                if token not in ('next','abs','min','max','toint','bool','count'):
                    raise ValueError(f'unsupported SMV function: {token}')
                left=('call',token,tuple(args))
            else: left=('name',token)
        else: raise ValueError(f'unexpected SMV token {token}')
        while True:
            op=tokens[index]
            if op=='?' and minimum==0:
                take('?'); yes=expression(); take(':'); no=expression()
                left=('case',((left,yes),(('literal',True),no))); continue
            level=PRECEDENCE.get(op,-1)
            if level<minimum: break
            take(); right=expression(level if op=='->' else level+1)
            left=('binary',op,left,right)
        return left
    result=expression()
    if tokens[index]!='<EOF>': raise ValueError(f'unconsumed SMV expression: {source[:200]}')
    return result


def decode(value):
    if value in ('TRUE','FALSE'): return value=='TRUE'
    return int(value) if re.fullmatch(r'-?\d+',str(value)) else value


def encode(value):
    return ('TRUE' if value else 'FALSE') if isinstance(value,bool) else str(value)


class Evaluation:
    def __init__(self,model,frames):
        self.model=model; self.frames=frames; self.cache={}; self.active=set()

    def name(self,name,tick):
        key=(name,tick)
        if key in self.cache: return self.cache[key]
        if name in self.model.defines:
            if key in self.active: raise ValueError(f'cyclic DEFINE {name}')
            self.active.add(key)
            value=self.eval(self.model.defines[name],tick)
            self.active.remove(key)
        elif name in self.model.variables:
            if tick>=len(self.frames): raise ValueError('next() beyond finite trace')
            if name not in self.frames[tick]: raise ValueError(f'missing state value: {name}')
            value=decode(self.frames[tick][name])
        elif name in self.model.constants: value=name
        else: raise ValueError(f'unknown SMV symbol: {name}')
        self.cache[key]=value
        return value

    def eval(self,node,tick=0):
        kind=node[0]
        if kind=='literal': return node[1]
        if kind=='name': return self.name(node[1],tick)
        if kind=='set': return frozenset(self.eval(n,tick) for n in node[1])
        if kind=='case':
            for condition,value in node[1]:
                if self.eval(condition,tick): return self.eval(value,tick)
            raise ValueError('case without a matching branch')
        if kind=='unary':
            value=self.eval(node[2],tick)
            return not value if node[1]=='!' else -value if node[1]=='-' else +value
        if kind=='call':
            name,args=node[1:]
            if name=='next':
                if len(args)!=1: raise ValueError('next arity')
                return self.eval(args[0],tick+1)
            values=[self.eval(a,tick) for a in args]
            return {'abs':abs,'min':min,'max':max,'toint':int,'bool':bool,
                    'count':lambda *x:sum(bool(v) for v in x)}[name](*values)
        op=node[1]; a=self.eval(node[2],tick)
        if op=='&': return bool(a) and bool(self.eval(node[3],tick))
        if op=='|': return bool(a) or bool(self.eval(node[3],tick))
        if op=='->': return not a or bool(self.eval(node[3],tick))
        b=self.eval(node[3],tick)
        if op in ('<->','xnor'): return bool(a)==bool(b)
        if op=='xor': return bool(a)!=bool(b)
        if op=='=': return a==b
        if op=='!=': return a!=b
        if op=='<': return a<b
        if op=='>': return a>b
        if op=='<=': return a<=b
        if op=='>=': return a>=b
        if op=='in': return a in b
        if op=='union': return frozenset(a)|frozenset(b)
        if op=='+': return a+b
        if op=='-': return a-b
        if op=='*': return a*b
        if op in ('/','mod','%'):
            # nuXmv integer division truncates toward zero.
            q=(abs(a)//abs(b)) * (-1 if (a<0)!=(b<0) else 1)
            return q if op=='/' else a-q*b
        raise ValueError(f'unsupported SMV operator: {op}')


class TraceModel:
    def __init__(self,text):
        self.variables=schema(text); self.defines={}; self.constants=set()
        self.constraints={'INIT':[],'INVAR':[],'TRANS':[]}; self.assignments=[]
        for kind,body in blocks(text):
            if kind=='DEFINE':
                for statement in statements(body):
                    name,value=statement[:-1].split(':=',1)
                    self.defines[name.strip()]=parse(value)
            elif kind=='CONSTANTS': self.constants.update(x.strip() for x in body.rstrip(';').split(','))
            elif kind in self.constraints:
                self.constraints[kind].append((body,parse(body.rstrip(';'))))
            elif kind=='ASSIGN':
                for statement in statements(body):
                    name,value=statement[:-1].split(':=',1); name=name.strip()
                    match=re.fullmatch(r'(init|next)\((.*)\)',name,re.S)
                    mode,target=match.groups() if match else ('current',name)
                    self.assignments.append((mode,target.strip(),parse(value)))
            elif kind in ('FAIRNESS','JUSTICE','COMPASSION'):
                raise ValueError(f'{kind} requires infinite-trace validation')
        self._temporal={}
        self.state_defines={n for n in self.defines if not n.startswith('__expr') and not self.temporal(('name',n))}

    def check_xml_import(self,path,output,timeout):
        """Check nuXmv reader compatibility; build no SMT/BDD solver."""
        output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=True)
        returned=output/'roundtrip.xml'; script=output/'import.cmd'
        script.write_text(f'read_model -i "{self.source}"\nflatten_hierarchy\nbuild_flat_model\n'
                          f'read_trace "{Path(path).resolve()}"\n'
                          f'show_traces -p 6 -o "{returned}" 1\nquit\n')
        result=execute(['nuXmv','-quiet','-source',str(script)],output,output/'import.log',timeout)
        if result['status']!='finished' or not returned.exists():
            raise ValueError(f'nuXmv XML import failed; inspect {output / "import.log"}')
        expected=states(path); actual=states(returned)
        if len(expected)!=len(actual): raise ValueError('nuXmv import changed the trace length')
        for tick,(a,b) in enumerate(zip(expected,actual)):
            for name in self.variables:
                if name not in b or decode(a[name])!=decode(b[name]):
                    raise ValueError(f'nuXmv import changed/missed {name} at tick {tick}')
            for name in self.state_defines & b.keys():
                if decode(a[name])!=decode(b[name]):
                    raise ValueError(f'nuXmv import changed DEFINE {name} at tick {tick}')
        result.update(valid=True,states=len(actual),solver_search=False)
        dump(output/'result.json',result)
        return result

    def temporal(self,node):
        kind=node[0]
        if kind=='name' and node[1] in self.defines:
            name=node[1]
            if name not in self._temporal: self._temporal[name]=self.temporal(self.defines[name])
            return self._temporal[name]
        if kind in ('literal','name'): return False
        if kind=='call': return node[1]=='next' or any(self.temporal(a) for a in node[2])
        if kind=='case': return any(self.temporal(a) or self.temporal(b) for a,b in node[1])
        if kind=='set': return any(self.temporal(a) for a in node[1])
        return any(self.temporal(a) for a in node[2:])

    def complete(self,frames,geometry):
        mapping={'local_to_global':list(range(len(geometry['sections'])))}
        completed=[handoff(f,self.variables,mapping) for f in frames]
        evaluation=Evaluation(self,completed)
        for tick,frame in enumerate(completed):
            for name in sorted(self.state_defines): frame[name]=encode(evaluation.name(name,tick))
        return completed

    def validate(self,frames):
        if not frames: raise ValueError('empty trace')
        expected=set(self.variables)|self.state_defines
        evaluation=Evaluation(self,frames)
        for tick,frame in enumerate(frames):
            missing=expected-frame.keys()
            if missing: raise ValueError(f'tick {tick}: missing symbols: {sorted(missing)[:8]}')
            for name,decl in self.variables.items():
                validate_value(name,frame[name],decl['domain'])
                if tick and decl['kind']=='FROZENVAR' and frame[name]!=frames[0][name]:
                    raise ValueError(f'tick {tick}: frozen value changed: {name}')
            for name in self.state_defines:
                if decode(frame[name])!=evaluation.name(name,tick):
                    raise ValueError(f'tick {tick}: incorrect DEFINE: {name}')
            for kind in ('INIT','INVAR','TRANS'):
                if kind=='INIT' and tick or kind=='TRANS' and tick==len(frames)-1: continue
                for source,expr in self.constraints[kind]:
                    if not evaluation.eval(expr,tick):
                        raise ValueError(f'tick {tick}: violated {kind}: {source[:240]}')
            for mode,name,expr in self.assignments:
                if mode=='init' and tick or mode=='next' and tick==len(frames)-1: continue
                value=evaluation.eval(expr,tick)
                actual=decode(frames[tick+(mode=='next')][name])
                valid=actual in value if isinstance(value,frozenset) else actual==value
                if not valid: raise ValueError(f'tick {tick}: violated {mode} assignment: {name}')
        return {'valid':True,'states':len(frames),'state_variables':len(self.variables),
                'state_defines':len(self.state_defines),'missing_symbols':0,
                'checked':'domains, frozen values, DEFINEs, INIT, INVAR, TRANS and ASSIGN'}
