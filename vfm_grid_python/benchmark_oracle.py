"""Property-blind, seeded enumeration of labelled partial grids, without repeats."""
import math
import random

from .common import grid_edges


def combination_at(n, k, rank):
    """Unrank lexicographic k-subsets without materializing all combinations."""
    selected, minimum = [], 0
    for left in range(k, 0, -1):
        for value in range(minimum, n):
            count = math.comb(n-value-1, left-1)
            if rank < count:
                selected.append(value)
                minimum = value+1
                break
            rank -= count
    return selected


def layouts(rows, columns, parkings, seed=0):
    """Affine permutation of edge-mask × parking-label assignments.

    Order is reproducible, but is not claimed to be a uniform shuffle of all
    permutations. Rejection checks only P degree one and no selected P-P edge.
    Connectivity, goals, roles, starts and the scenario are deliberately absent.
    Memory usage is independent of the number of visited assignments.
    """
    n = rows*columns
    if rows < 1 or columns < 1 or not 0 <= parkings <= n:
        raise ValueError("invalid oracle dimensions/parking count")
    edges = grid_edges(rows, columns)
    masks = 1 << len(edges)
    total = masks * math.comb(n, parkings)
    rng = random.Random(seed)
    stride = rng.randrange(1, total) if total > 1 else 1
    while math.gcd(stride, total) != 1:
        stride = rng.randrange(1, total)
    offset = rng.randrange(total)
    for attempt in range(total):
        index = (offset + attempt*stride) % total
        labels = set(combination_at(n, parkings, index//masks))
        mask = index % masks
        selected = [edge for bit, edge in enumerate(edges) if mask & (1 << bit)]
        degree = [0]*n
        if any(u in labels and v in labels for u,v in selected):
            continue
        for u,v in selected:
            degree[u] += 1
            degree[v] += 1
        if any(degree[p] != 1 for p in labels):
            continue
        yield dict(rows=rows, columns=columns,
                   nodes=[dict(id=u, row=u//columns, column=u%columns,
                               kind="P" if u in labels else "N") for u in range(n)],
                   edges=[list(e) for e in selected], oracle_assignment=str(index),
                   oracle_assignments_examined=attempt+1)
