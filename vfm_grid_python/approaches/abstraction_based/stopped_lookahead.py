"""Stopped lookahead targets, with positions free within paper milestone cells."""
from vfm_grid_python.common import both
from vfm_grid_python.concrete.observations import road_mapping
from .paper_cells import milestone


def build_targets(frames, config, geometry):
    """Keep cell/rank/property progress and require zero terminal velocities.

    Connections are allowed. These targets constrain the lookahead tail, not
    each committed handoff. All original concrete dynamics remain in place.
    """
    mapping = road_mapping(config, geometry)
    stopped = [f'env.veh___6{i}9___.v = 0' for i in range(config['cars'])]
    return [both([milestone(frame, config, mapping), *stopped]) for frame in frames], dict(
        mode='stopped', position_policy='free_within_milestone_cells',
        initial_target_enforced=True, guaranteed_greedy_refinement=False,
        note='A feasible stopped continuation is witnessed, but no canonical '
             'placement or universally extendible stopped-state set is certified.')
