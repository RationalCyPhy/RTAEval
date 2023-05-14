import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('acc/acc_utils/accRTA.py', '')
sys.path.append(baseDirPath)

from demo_rta.reachRTA import reachRTA, convert_trace_to_rectangles, createInitState
# from verse import Scenario

from demo_rta.simRTA import simRTA

class accSimRTA(simRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt=0.1, T=0.5):
        super().__init__(egoAgent, agents, unsafeSets, egoModes, otherModes, dt, T)
        self.setupEval(egoAgent.id, 1)
        self.c = 7

    def RTALogic(self, simulationTrace: dict):
        predictedTraj = self.simulate_forward(simulationTrace)

        egoTrace = predictedTraj['agents'][self.egoAgent.id]['state_trace']

        for unsafeSet in self.unsafeSets:
            unsafeSetTrace = predictedTraj['unsafe'][unsafeSet.id]['state_trace']

            for i in range(len(unsafeSetTrace)):
                egoPos = egoTrace[i][1]
                unsafeSetDef = unsafeSetTrace[i][1]


                pos_max = unsafeSetDef[0][0] - unsafeSetDef[1]
                if egoPos > pos_max:
                    return self.egoModes.SAFETY

        for agent in self.agents:
            if agent.id != self.egoID:
                agentTrace = predictedTraj['agents'][agent.id]['state_trace']
                for i in range(len(agentTrace)):
                    agentPos = agentTrace[i][1]
                    egoPos = egoTrace[i][1]
                    if abs(egoPos - agentPos) <= self.c:
                        return self.egoModes.SAFETY
                        
        return super().RTALogic(simulationTrace)


class accReachRTA(reachRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt=0.05, T=0.15):
        super().__init__(egoAgent, agents, unsafeSets, egoModes, otherModes, dt, T)
        self.setupEval(egoAgent.id, 1)
        self.c = 7
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