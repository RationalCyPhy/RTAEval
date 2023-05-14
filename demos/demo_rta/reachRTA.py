import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/demo_rta/reachRTA.py', '')
sys.path.append(baseDirPath)

import time
import copy

from utils.baseRTA import baseRTA

from verse import Scenario

def convert_trace_to_rectangles(trace, trace_name: str):
    '''
    Takes a trace from verse and converts it to a list of rectangles
    '''
    rectangles = []
    trace_rectangles = trace.nodes[0].trace[trace_name]
    for i in range(0, len(trace_rectangles), 2):
        rectangles.append((trace_rectangles[i], trace_rectangles[i + 1]))
    return rectangles

def createInitState(state, dState):
    try:
        minState = [state[i]-dState[i] for i in range(len(state))]
        maxState = [state[i]+dState[i] for i in range(len(state))]
    except:
        minState = [state[i]-dState for i in range(len(state))]
        maxState = [state[i]+dState for i in range(len(state))]
    return [minState, maxState]

class reachRTA(baseRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt = 0.1, T = 0.5):
        super().__init__()
        self.egoAgent = copy.deepcopy(egoAgent)
        self.otherAgents = copy.deepcopy(agents)
        self.unsafeSets = copy.deepcopy(unsafeSets)
        self.dt = 0.1
        self.T = 0.5
        self.egoModeClass = egoModes
        self.otherModes = otherModes
        self.dState = 1
        self.c = 100

    def getAgentRects(self, agent, initStates, initMode):
        scenario = Scenario()
        scenario.add_agent(agent)
        scenario.set_init([initStates], [tuple([initMode])])
        traces = scenario.verify(self.T, self.dt)
        rects = convert_trace_to_rectangles(traces, agent.id)
        return rects

    def checkRect(self, egoRect, rect2):
        egoRect_min, egoRect_max = egoRect
        rect2_min, rect2_max = rect2

        for i in range(len(egoRect_min)):
            try:
                if egoRect_max[i] < rect2_min[i] or egoRect_min[i] > rect2_max[i]:
                    return False
            except:
                pass
        return True

    def RTALogic(self, simulationTrace):
        egoState = simulationTrace['agents'][self.egoAgent.id]['state_trace'][-1]
        egoTime = egoState[0]
        egoState = egoState[1:]

        if egoTime == 0:
            egoMode = simulationTrace['agents'][self.egoAgent.id]['mode_trace'][-1]

        self.egoAgent.predictedSimulation = simulationTrace
        initStates = createInitState(egoState, self.dState)

        egoRectangles = self.getAgentRects(self.egoAgent,initStates,self.egoModeClass.UNTRUSTED)


        otherAgentRects = []

        for i in range(len(self.otherAgents)):
            currAgent = self.otherAgents[i]
            currMode = self.otherModes[i]
            try:
                currState = simulationTrace['agents'][currAgent.id]['state_trace'][-1]
                currTime = currState[0]
                currState = currState[1:]
                currAgent.predictedSimulation = simulationTrace
            except:
                currState = simulationTrace['agents'][currAgent.id]['state_trace'][-1]
                currTime = currState[0]
                currState = currState[1:]
            
            initStates = createInitState(currState, self.c)
            currRectangles = self.getAgentRects(currAgent, initStates, currMode)
            otherAgentRects.append(currRectangles)
            
        collision = False
        for egoRect in egoRectangles:
            for agentRects in otherAgentRects:
                for otherRect in agentRects:
                    collision = self.checkRect(egoRect, otherRect)
                    if collision:
                        return self.egoModeClass.SAFETY
            for unsafeSet in self.unsafeSets:
                if unsafeSet.set_type == "rectangle":
                    center, dx, dy, dz = unsafeSet.update_def(simulationTrace)
                    x_min = center[0] - (dx/2+self.c)
                    x_max = center[0] + (dx/2+self.c)
                    y_min = center[1] - (dy/2+self.c)
                    y_max = center[1] + (dy/2+self.c)
                    z_min = center[2] - (dz/2+self.c)
                    z_max = center[2] + (dz/2+self.c)

                    rect = ([egoRect[0][0], x_min,y_min,z_min], [egoRect[1][0], x_max, y_max, z_max])
                    collision = self.checkRect(egoRect, rect)
                    if collision:
                        return self.egoModeClass.SAFETY

        return self.egoModeClass.UNTRUSTED