from dataclasses import dataclass
from enum import Enum, IntEnum


class EndEffectorSide(Enum):
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


@dataclass
class EndEffectorState:
    class GraspingState(IntEnum):
        ERROR = -1
        UNAVAILABLE = -2
        UNKNOWN = 0
        MOVING = 1
        REACHED = 2
        GRABBED = 3

    position: list
    velocity: list
    effort: list
    state: GraspingState
