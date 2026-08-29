from .approach_0 import run as approach_0
from .approach_1 import run as approach_1
from .approach_2 import run as approach_2
from .approach_3 import run as approach_3

APPROACHES = {
    0: approach_0,
    1: approach_1,
    3: approach_3,
}

__all__ = ["APPROACHES", "approach_0", "approach_1", "approach_2", "approach_3"]
