import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/dubins/dubins_gcas.py', '')
sys.path.append(baseDirPath)

import time

from utils.simpleSim import simpleSim
from utils.unsafeSets import *

from dubins_utils.agent_aircraft import *
from dubins_utils.controller_aircraft import *
from dubins_utils.dubinsRTA import dubinsSimRTA, dubinsReachRTA

controllerFile = curr_path.replace('dubins_gcas.py', 'dubins_utils/controller_aircraft.py')

def ego1_desiredTraj(simulationTrace):
    d_behind = 100
    d_in = 0
    d_under = 100
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2] - d_under, lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego1_safeTraj(simulationTrace):
    d_behind = 100
    d_in = 0
    d_under = 0
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2] - d_under, lead_state[3], lead_state[4], lead_state[5]]
    return goalState

init_Trace = {'agents':{'ego1':{'state_trace':[[0,-75,-75,0,pi/2,0,90]]}, 'other1':{'state_trace':[[0,100,0,0,0,0,100]]}}}


egoAgent1 = AircraftTrackingAgent("ego1",(ego1_desiredTraj,init_Trace, ego1_safeTraj),file_name=controllerFile)

egoInit = [-75,-75,0,0,0,90]

otherAgent1 = AircraftAgent('other1', ([pi/18,0,0],),file_name=controllerFile)
otherInit = [0,0,0,0,0,100]

unsafe1 = relativeUnsafeBall("leader_ball", [0,0,0], 50, "other1")


ground = pc.qhull(np.array([[-750, -100, -200],
           [-750, -100, -150],
           [750, -100, -200],
           [750, -100, -150],
           [750, 1400, -200],
           [750, 1400, -50],
           [-750, 1400, -200],
           [-750, 1400, -50]]))

A_ground = [list(row) for row in ground.A]
b_ground = list(ground.b)
unsafe2 = staticUnsafePolytope("ground", A_ground, b_ground)

egoRTA = dubinsReachRTA(egoAgent1, [otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL])
rtas = [egoRTA, None]


agents = [egoAgent1, otherAgent1]
modes = [AircraftMode.UNTRUSTED, AircraftMode.NORMAL]
initStates = [egoInit, otherInit]

dubinsSim = simpleSim()
dubinsSim.setSimType(vis=False, plotType="3D", simType="3D")
dubinsSim.setTimeParams(dt=0.05, T=40)
dubinsSim.addAgents(agents=agents, RTAs=rtas, modes=modes, initStates=initStates)
dubinsSim.addUnsafeSets(unsafe_sets = [unsafe1, unsafe2])
dubinsSim.chooseAxesLimits("fixed", "other1", [-750,750], [-100,1400], [-750,750])

start_time = time.time()
simulationTrace = dubinsSim.runSim()
exec_time = time.time() - start_time

start_time = time.time()
egoRTA.eval.summary()
egoRTA.eval.compute_unsafe_TTC(-1, 3, 4)
egoRTA.eval.compute_agent_TTC(-1, 3, 4)
print('Evaluation time: ', time.time() - start_time)

untrusted_usage, safety_usage, num_switches = egoRTA.eval.controllerUsage()
averageRT = sum(egoRTA.eval.computation_times)/(len(egoRTA.eval.computation_times))
print('Average Computation time: ', averageRT)
print('Num switches: ', num_switches)
print('Execution time: ', exec_time)
