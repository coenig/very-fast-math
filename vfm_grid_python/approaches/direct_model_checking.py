"""One concrete VFM query selecting topology and scenario symbolically."""
from vfm_grid_python.concrete.compiler import run as check_concrete


def run(config, output, bound=200, timeout=300):
    return check_concrete(config, None, output, bound, timeout)
