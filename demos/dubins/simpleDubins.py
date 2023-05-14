import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/dubins/simpleDubins.py', '')
sys.path.append(baseDirPath)

from utils.simpleSim import simpleSim
from utils.unsafeSets import *

from dubins_utils.agent_aircraft import *
from dubins_utils.controller_aircraft import *
from dubins_utils.dubinsRTA import dubinsSimRTA, dubinsReachRTA

import time

controllerFile = curr_path.replace('simpleDubins.py', 'dubins_utils/controller_aircraft.py')

def ego1_desiredTraj(simulationTrace):
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 100*sin(lead_state[3]), lead_state[1] + 100*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego1_safeTraj(simulationTrace):
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 200*sin(lead_state[3]), lead_state[1] + 200*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]*0.5]
    return goalState

init_Trace = init_trace = {'agents':{'ego1':{'state_trace':[[0,-75,-75,0,pi/2,0,90]]}, 'other1':{'state_trace':[[0,100,0,0,0,0,100]]}}}


egoAgent1 = AircraftTrackingAgent("ego1",(ego1_desiredTraj,init_Trace, ego1_safeTraj),file_name=controllerFile)

egoInit = [-75,-75,0,0,0,90]

otherAgent1 = AircraftAgent('other1', ([pi/18,0,0],),file_name=controllerFile)
otherInit = [0,0,0,0,0,100]

unsafe1 = relativeUnsafeBall("leader_ball", [0,0,0], 50, "other1")
unsafe2 = staticUnsafeRectangle("building",[470,570,0], 75, 75, 50)

egoRTA = dubinsReachRTA(egoAgent1, [otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL])
rtas = [egoRTA, None]


agents = [egoAgent1, otherAgent1]
# rtas = [None, None]
modes = [AircraftMode.UNTRUSTED, AircraftMode.NORMAL]
initStates = [egoInit, otherInit]

dubinsSim = simpleSim()
dubinsSim.setSimType(vis=False, plotType="2D", simType="3D")
dubinsSim.setTimeParams(dt=0.05, T=20)
dubinsSim.addAgents(agents=agents, RTAs=rtas, modes=modes, initStates=initStates)
dubinsSim.addUnsafeSets(unsafe_sets = [unsafe1, unsafe2])
dubinsSim.chooseAxesLimits("fixed", "other1", [-750,750], [-100,1400], [-750,750])

start_time = time.time()
simulationTrace = dubinsSim.runSim()
exec_time = time.time() - start_time

start_time = time.time()
egoRTA.eval.summary(True)
egoRTA.eval.compute_unsafe_TTC(-1, 3, 4)
egoRTA.eval.compute_agent_TTC(-1, 3, 4)
print('Evaluation time: ', time.time() - start_time)

untrusted_usage, safety_usage, num_switches = egoRTA.eval.controllerUsage()

averageRT = sum(egoRTA.eval.computation_times)/(len(egoRTA.eval.computation_times))

print('Average Computation time: ', averageRT)
print('Num switches: ', num_switches)
print('Execution time: ', exec_time)
