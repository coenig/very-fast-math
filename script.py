"""Compatibility imports for the former monolithic entry script.

New code should invoke ``main.py`` and import approaches from ``approaches``.
"""

from approaches import approach_0, approach_1, approach_2, approach_3
from approaches.approach_3 import (
    add_embedding_lengths,
    exact_graph_init,
    iter_section_planar_graphs,
    read_smv_graph_domain,
)
from graph_tools import (
    GraphBuilder,
    dump_graph,
    dump_graph_with_lengths,
    geometric_length_graph,
)
from main import main
from model_checking import Morty, NuXmvConn
from scenario_generation import (
    get_param_simple,
    get_parameters_simplest_geometries,
    get_parameters_test_enumeration,
    parametrize_model_config as parametrize_model_config_with_dict,
)

__all__ = [
    "GraphBuilder",
    "Morty",
    "NuXmvConn",
    "approach_0",
    "approach_1",
    "approach_2",
    "approach_3",
    "add_embedding_lengths",
    "dump_graph",
    "dump_graph_with_lengths",
    "geometric_length_graph",
    "get_param_simple",
    "get_parameters_simplest_geometries",
    "get_parameters_test_enumeration",
    "main",
    "exact_graph_init",
    "iter_section_planar_graphs",
    "parametrize_model_config_with_dict",
    "read_smv_graph_domain",
]


if __name__ == "__main__":
    raise SystemExit(main())
