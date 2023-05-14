import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('dubins/dubins_utils/dubinsRTA.py', '')
sys.path.append(baseDirPath)

from demo_rta.reachRTA import reachRTA, convert_trace_to_rectangles, createInitState
from verse import Scenario
from math import sqrt
import numpy as np

from demo_rta.simRTA import simRTA
from demo_rta.reachRTA import reachRTA

import cvxpy as cp
from yices import *

class dubinsSimRTA(simRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt=0.05, T=0.15):
        super().__init__(egoAgent, agents, unsafeSets, egoModes, otherModes, dt, T)
        self.setupEval(egoAgent.id, 3)
        self.c = 100

    def RTALogic(self, simulationTrace: dict):
        predictedTraj = self.simulate_forward(simulationTrace)

        egoTrace = predictedTraj['agents'][self.egoAgent.id]['state_trace']

        for unsafeSet in self.unsafeSets:
            unsafeSetTrace = predictedTraj['unsafe'][unsafeSet.id]['state_trace']

            for i in range(len(unsafeSetTrace)):
                egoPos = egoTrace[i][1:]
                unsafeSetDef = unsafeSetTrace[i][1]
                if unsafeSet.num_dims == 2:
                    curr_state = [egoPos[0], egoPos[1]]
                elif unsafeSet.num_dims == 3:
                    curr_state = [egoPos[0], egoPos[1], egoPos[2]]

                # Check for safety here:
                if unsafeSet.set_type == "point":
                    if unsafeSetDef[0] == curr_state[0] and unsafeSetDef[1] == curr_state[1] and unsafeSetDef[2] == curr_state[2]:
                        return self.egoModes.SAFETY
                        
                elif unsafeSet.set_type == "ball":
                    center, r = unsafeSetDef
                    dist_from_center = np.linalg.norm(np.array(center) - np.array(curr_state))
                    if dist_from_center < r:
                        return self.egoModes.SAFETY

                elif unsafeSet.set_type == "rectangle":
                    center = unsafeSetDef[0]
                    dx = unsafeSetDef[1]
                    dy = 0 if unsafeSet.num_dims < 2 else unsafeSetDef[2]
                    dz = 0 if unsafeSet.num_dims < 3 else unsafeSetDef[3]

                    x_min = center[0] - (dx/2 + self.c)
                    x_max = center[0] + (dx/2 + self.c)

                    y_min = center[1] - (dy/2 + self.c)
                    y_max = center[1] + (dy/2 + self.c)

                    z_min = center[2] - (dz/2 + self.c)
                    z_max = center[2] + (dz/2 + self.c)


                    if x_min <= curr_state[0] <= x_max and y_min <= curr_state[1] <= y_max and z_min <= curr_state[2] <= z_max:
                        return self.egoModes.SAFETY
                
                elif unsafeSet.set_type == "polytope":
                    A = unsafeSetDef[0]
                    b = unsafeSetDef[1]

                    in_face = [False for row in range(len(b))]
                    for row in range(len(b)):
                        is_in = A[row][0]*curr_state[0] + A[row][1]*curr_state[1] +A[row][2]*curr_state[2] <= b[row]
                        in_face[row] = is_in

                    if False not in in_face:
                        return self.egoModes.SAFETY
    
                else:
                    A = unsafeSetDef[0]
                    b = unsafeSetDef[1]
                                    
                    resulting_vec = np.matmul(A, np.array(curr_state))
                    for i in range(len(resulting_vec)):
                        check_if_in = [resulting_vec[i] <= b[i] + self.c for i in range(len(resulting_vec))]
                    if False not in check_if_in:
                        return self.egoModes.SAFETY

        for agent in self.agents:
            if agent.id != self.egoID:
                agentTrace = predictedTraj['agents'][agent.id]['state_trace']
                for i in range(len(agentTrace)):
                    agentPos = [agentTrace[i][1], agentTrace[i][2], agentTrace[i][3]]
                    egoPos = [egoTrace[i][1], egoTrace[i][2], egoTrace[i][3]]
                    if sqrt((agentPos[0] - egoPos[0])**2 + (agentPos[1] - egoPos[1])**2 + (agentPos[2] - egoPos[2])**2) <= self.c:
                        return self.egoModes.SAFETY

        return self.egoModes.UNTRUSTED

class dubinsReachRTA(reachRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt=0.05, T=0.15):
        super().__init__(egoAgent, agents, unsafeSets, egoModes, otherModes, dt, T)
        self.setupEval(egoAgent.id, 3)
        self.c = 100

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
            
            initStates = createInitState(currState, self.dState)
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

                elif unsafeSet.set_type == 'polytope':
                    A, b = unsafeSet.update_def(simulationTrace)
                    x_min = egoRect[0][1]
                    y_min = egoRect[0][2]
                    z_min = egoRect[0][3]
                    x_max = egoRect[1][1]
                    y_max = egoRect[1][2]
                    z_max = egoRect[1][3]

                    cfg = Config()
                    cfg.default_config_for_logic('QF_LRA')
                    ctx = Context(cfg)

                    real_t = Types.real_type()
                    x = Terms.new_uninterpreted_term(real_t, 'x')
                    y = Terms.new_uninterpreted_term(real_t, 'y')
                    z = Terms.new_uninterpreted_term(real_t, 'z')

                    fmlas = []

                    for row in range(len(b)):
                        str1 = '(* '+str(A[row][0])+' x)'
                        str2 = '(* '+str(A[row][1])+' y)'
                        str3 = '(* '+str(A[row][2])+' z)'
                        str123 = '(+ (+ '+str1+' '+str2+') '+str3+')'
                        fmlas.append(Terms.parse_term('(<= '+str123+' '+str(b[row])+')'))

                    fmlas.append(Terms.parse_term('(<= '+str(x_min)+' x)'))
                    fmlas.append(Terms.parse_term('(<= '+str(y_min)+' y)'))
                    fmlas.append(Terms.parse_term('(<= '+str(z_min)+' z)'))

                    fmlas.append(Terms.parse_term('(>= '+str(x_max)+' x)'))
                    fmlas.append(Terms.parse_term('(>= '+str(y_max)+' y)'))
                    fmlas.append(Terms.parse_term('(>= '+str(z_max)+' z)'))

                    ctx.assert_formulas(fmlas)

                    status = ctx.check_context()
                    if status == Status.SAT:
                        return self.egoModeClass.SAFETY

                elif unsafeSet.set_type == "ball":
                    center, r = unsafeSet.update_def(simulationTrace)
                    x_min = egoRect[0][1]
                    y_min = egoRect[0][2]
                    z_min = egoRect[0][3]
                    x_max = egoRect[1][1]
                    y_max = egoRect[1][2]
                    z_max = egoRect[1][3]

                    x = cp.Variable(1)
                    y = cp.Variable(1)
                    z = cp.Variable(1)

                    objective = cp.Minimize((x - center[0])**2 + (y - center[1])**2 + (z - center[2])**2)
                    constraints = [x <= x_max, y <= y_max, z <= z_max, x >= x_min, y >= y_min, z >= z_min]
                    prob = cp.Problem(objective, constraints)
                    result = prob.solve()

                    if sqrt(result) <= r:
                        return self.egoModeClass.SAFETY



        return self.egoModeClass.UNTRUSTED
