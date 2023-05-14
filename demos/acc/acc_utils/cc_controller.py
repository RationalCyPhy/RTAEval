'''
This is mode controller for 3D dubins aircraft allows for
transitions between the aircraft modes and defines the aircraft state
'''
from enum import Enum, auto
import copy

class ccMode(Enum):
    '''Defines the discrete modes of a single agent'''
    NORMAL = auto()
    UNTRUSTED = auto()
    SAFETY = auto()


class State:
    '''Defines the state variables of the model
        Both discrete and continuous variables.
        Initial values defined here do not matter.
    '''
    mode: ccMode

    x = 0
    v = 0

    def __init__(self, x, v, cc_mode):
        pass



def decisionLogic(ego: State, others: State):
    '''Computes the possible mode transitions
        For now this is an empty controller function.
        Coming soon. Waypoint transitions. Platooning.'''
    output = copy.deepcopy(ego)
    return output
