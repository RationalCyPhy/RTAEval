import abc
from enum import Enum

import time

from utils.RTAEval import RTAEval

class baseRTA(abc.ABC):
    def __init__(self):
        # This is the requirements for initializing the RTA
        self.do_eval = False
        self.egoID = None
        pass

    @abc.abstractmethod
    def RTALogic(self, simulationTrace: dict):
        # method for switching RTA
        pass

    def RTASwitch(self, simulationTrace: dict):
        start_time = time.time()
        rtaMode = self.RTALogic(simulationTrace)
        running_time = time.time() - start_time

        if self.do_eval:
            self.eval.collect_computation_time(running_time)
            self.eval.collect_trace(simulationTrace)
        return rtaMode

    def setupEval(self, egoID, workspaceDims):
        self.do_eval = True
        self.eval = RTAEval()
        self.eval.egoID = egoID
        self.eval.workspaceDims = workspaceDims
