"""Equivalent finite-cell encodings; opt-in, preserving road/cell observations."""
from . import paper_cells, abstract
from vfm_grid_python.scenarios.roads import inventory

VARIANTS = ("baseline", "folded_counts", "flat_position", "flat_adjacency",
            "relational_counts", "relational")


def layout(config):
    roads, _ = inventory(config)
    body, _ = paper_cells.dimensions(config)
    lengths = paper_cells.road_lengths(config)
    counts = {}
    for r in roads:
        ls = ([config['section_length'], config['parking_length']]
              if r['kind'] == 'section' else [lengths[r['id']]['length']])
        counts[r['id']] = tuple(paper_cells.CellDomain(n, body).count for n in ls)
    positions = [(r['id'], c) for r in roads for c in range(max(counts[r['id']]))]
    return roads, counts, positions


def count_expression(road, counts):
    ns = counts[road['id']]
    if len(set(ns)) == 1:
        return str(ns[0])
    u, v = road['edge']
    return f'case parking_{u} | parking_{v} : {ns[1]}; TRUE : {ns[0]}; esac'


def adjacency(roads, counts, positions):
    """Candidate table: (destination ID, required source/destination counts).

    Endpoint counts remain guarded by the symbolic parking labels. Active-road
    and valid-cell invariants are retained verbatim from the baseline model.
    """
    table = {}
    for p, (r, c) in enumerate(positions):
        choices = []
        for q, (s, d) in enumerate(positions):
            if r == s:
                if abs(c-d) <= 1:
                    choices.append((q, None, None))
                continue
            if roads[r]['kind'] == roads[s]['kind']:
                continue
            for j in set(roads[r]['joints']) & set(roads[s]['joints']):
                side_r = roads[r]['joints'].index(j)
                side_s = roads[s]['joints'].index(j)
                if ((side_r == 0 and c != 0) or (side_s == 0 and d != 0)
                        or (side_r == 1 and c+1 not in counts[r])
                        or (side_s == 1 and d+1 not in counts[s])):
                    continue
                choices.append((q, c+1 if side_r else None, d+1 if side_s else None))
        table[p] = choices
    return table


def rewrite(text, config, variant):
    if variant not in VARIANTS:
        raise ValueError(variant)
    if variant == 'baseline':
        return text
    roads, counts, positions = layout(config)
    body, _ = paper_cells.dimensions(config)
    for i in range(config['cars']):
        old = f'DEFINE ncells_{i} := (ilen_{i} + {body-1}) / {body};'
        new = f'DEFINE ncells_{i} := case ' + ' '.join(
            f'road_{i} = {r["id"]} : {count_expression(r, counts)};' for r in roads) + ' TRUE : 1; esac;'
        assert text.count(old) == 1
        text = text.replace(old, new)
    if variant == 'folded_counts':
        return text
    largest = max(max(ns) for ns in counts.values())
    for i in range(config['cars']):
        old = f' road_{i} : 0..{len(roads)-1};'
        assert text.count(old) == 1
        text = text.replace(old, f' position_{i} : 0..{len(positions)-1};')
        old = f'VAR cell_{i} : 0..{largest-1};'
        assert text.count(old) == 1
        definitions = []
        for name, component in [('road', 0), ('cell', 1)]:
            definitions.append(f'DEFINE {name}_{i} := case ' + ' '.join(
                f'position_{i} = {p} : {rc[component]};' for p, rc in enumerate(positions))
                + ' TRUE : 0; esac;')
        text = text.replace(old, '\n'.join(definitions))
    if variant == 'flat_position':
        return text
    table = adjacency(roads, counts, positions)
    for i in range(config['cars']):
        old = abstract.transfer_constraint(roads, i, 'joint').replace(
            f'(next(road_{i}) = road_{i})',
            f'(next(road_{i}) = road_{i} & abs(next(cell_{i}) - cell_{i}) <= 1)')
        assert text.count(old) == 1
        rules = []
        for p, choices in table.items():
            terms = []
            for q, nr, ns in choices:
                terms.append('(' + ' & '.join(
                    [f'next(position_{i}) = {q}']
                    + ([f'ncells_{i} = {nr}'] if nr else [])
                    + ([f'next(ncells_{i}) = {ns}'] if ns else [])) + ')')
            rules.append(f'TRANS position_{i} = {p} -> (' + ' | '.join(terms) + ');')
        text = text.replace(old, '\n'.join(rules))
    if variant in ('relational_counts', 'relational'):
        # Do not inline the selected road's symbolic cell count into every
        # endpoint/transition expression. A numeric case over all roads and
        # parking flags can cause exponential ADD growth in scalar encoding.
        # Each auxiliary value is uniquely determined by the selected road;
        # INVAR also constrains it in successor states. Projecting it away
        # therefore recovers exactly the flat_adjacency transition system.
        for i in range(config['cars']):
            old = f'DEFINE ncells_{i} := case ' + ' '.join(
                f'road_{i} = {r["id"]} : {count_expression(r, counts)};'
                for r in roads) + ' TRUE : 1; esac;'
            assert text.count(old) == 1
            lower = min(min(ns) for ns in counts.values())
            upper = max(max(ns) for ns in counts.values())
            constraints = [f'VAR ncells_{i} : {lower}..{upper};']
            constraints += [
                f'INVAR road_{i} = {r["id"]} -> '
                f'ncells_{i} = ({count_expression(r, counts)});'
                for r in roads]
            text = text.replace(old, '\n'.join(constraints))
    if variant == 'relational':
        # layout() is a bijection between valid position IDs and (road,cell).
        # Compare IDs directly instead of building two decoded comparisons.
        for i in range(config['cars']):
            for j in range(config['cars']):
                if i == j:
                    continue
                text = text.replace(
                    f'road_{i} = road_{j} & cell_{i} = cell_{j}',
                    f'position_{i} = position_{j}')
                text = text.replace(
                    f'next(road_{i}) = road_{j} & next(cell_{i}) = cell_{j}',
                    f'next(position_{i}) = position_{j}')
    return text
