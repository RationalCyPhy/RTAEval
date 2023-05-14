import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/dubins/three_simpleDubins.py', '')
sys.path.append(baseDirPath)

from utils.simpleSim import simpleSim
from utils.unsafeSets import *

from dubins_utils.agent_aircraft import *
from dubins_utils.controller_aircraft import *
from dubins_utils.dubinsRTA import dubinsSimRTA, dubinsReachRTA

import time

controllerFile = curr_path.replace('three_simpleDubins.py', 'dubins_utils/controller_aircraft.py')

def ego1_desiredTraj(simulationTrace):
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 100*sin(lead_state[3]), lead_state[1] + 100*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego1_safeTraj(simulationTrace):
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 200*sin(lead_state[3]), lead_state[1] + 200*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]*0.5]
    return goalState


def ego2_desiredTraj(simulationTrace):
    lead_state = simulationTrace['agents']['ego1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 75*sin(lead_state[3]), lead_state[1] + 75*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    
    
    return goalState

def ego2_safeTraj(simulationTrace):
    lead_state = simulationTrace['agents']['ego1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 150*sin(lead_state[3]), lead_state[1] + 150*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]*0.5]
    return goalState

def ego3_desiredTraj(simulationTrace):
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] + 100*sin(lead_state[3]), lead_state[1] - 75*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    
    
    return goalState

def ego3_safeTraj(simulationTrace):
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] + 150*sin(lead_state[3]), lead_state[1] - 150*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]*0.5]
    return goalState



init_Trace = init_trace = {'agents':{'ego1':{'state_trace':[[0,-75,-75,0,pi/2,0,90]]}, 'leader':{'state_trace':[[0,100,0,0,0,0,100]]}}}


egoAgent1 = AircraftTrackingAgent("ego1",(ego1_desiredTraj,init_Trace, ego1_safeTraj),file_name=controllerFile)
egoAgent2 = AircraftTrackingAgent("ego2",(ego2_desiredTraj,init_Trace, ego2_safeTraj),file_name=controllerFile)
egoAgent3 = AircraftTrackingAgent("ego3",(ego3_desiredTraj,init_Trace, ego3_safeTraj),file_name=controllerFile)


ego1Init = [-75,-75,0,0,0,90]
ego2Init = [-150,-150,0,0,0,90]
ego3Init = [0,-200,0,0,0,90]


otherAgent1 = AircraftAgent('leader', ([pi/18,0,0],),file_name=controllerFile)
otherInit = [0,0,0,0,0,100]

unsafe1 = relativeUnsafeBall("leader_ball", [0,0,0], 50, "leader")
unsafe2 = staticUnsafeRectangle("building",[470,570,0], 75, 75, 50)

ego1RTA = dubinsReachRTA(egoAgent1, [egoAgent2, egoAgent3, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego2RTA = dubinsReachRTA(egoAgent2, [egoAgent1, egoAgent3, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego3RTA = dubinsReachRTA(egoAgent3, [egoAgent1, egoAgent2, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])

rtas = [ego1RTA, ego2RTA, ego3RTA, None]


agents = [egoAgent1, egoAgent2, egoAgent3, otherAgent1]
# rtas = [None, None]
modes = [AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.NORMAL]
initStates = [ego1Init, ego2Init, ego3Init, otherInit]

dubinsSim = simpleSim()
dubinsSim.setSimType(vis=False, plotType="2D", simType="3D")
dubinsSim.setTimeParams(dt=0.05, T=20)
dubinsSim.addAgents(agents=agents, RTAs=rtas, modes=modes, initStates=initStates)
dubinsSim.addUnsafeSets(unsafe_sets = [unsafe1, unsafe2])
dubinsSim.chooseAxesLimits("fixed", "leader", [-750,750], [-200,1300], [-100,100])

start_time = time.time()
simulationTrace = dubinsSim.runSim()
exec_time = time.time() - start_time

start_time = time.time()
ego1RTA.eval.summary(True)
ego1RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego1RTA.eval.compute_agent_TTC(-1, 3, 4)

ego2RTA.eval.summary()
ego2RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego2RTA.eval.compute_agent_TTC(-1, 3, 4)


ego3RTA.eval.summary()
ego3RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego3RTA.eval.compute_agent_TTC(-1, 3, 4)
print('Evaluation time: ', time.time() - start_time)

untrusted_usage, safety_usage, num_switches1 = ego1RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches2 = ego2RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches3 = ego3RTA.eval.controllerUsage()

averageRT1 = sum(ego1RTA.eval.computation_times)/(len(ego1RTA.eval.computation_times))
averageRT2 = sum(ego2RTA.eval.computation_times)/(len(ego2RTA.eval.computation_times))
averageRT3 = sum(ego3RTA.eval.computation_times)/(len(ego3RTA.eval.computation_times))

print('Average Computation time: ', (averageRT1+averageRT2+averageRT3)/3)
print('Num switches: ', num_switches1+num_switches2+num_switches3)
print('Execution time: ', exec_time)