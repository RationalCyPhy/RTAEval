import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/dubins/ten_simpleDubins.py', '')
sys.path.append(baseDirPath)

from utils.simpleSim import simpleSim
from utils.unsafeSets import *

from dubins_utils.agent_aircraft import *
from dubins_utils.controller_aircraft import *
from dubins_utils.dubinsRTA import dubinsSimRTA, dubinsReachRTA

import time

controllerFile = curr_path.replace('ten_simpleDubins.py', 'dubins_utils/controller_aircraft.py')

def ego1_desiredTraj(simulationTrace):
    d_behind = 0
    d_in = 100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego1_safeTraj(simulationTrace):
    d_behind = 0
    d_in = 200
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego2_desiredTraj(simulationTrace):
    d_behind = 0
    d_in = 75
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState
    
    
def ego2_safeTraj(simulationTrace):
    d_behind = 0
    d_in = 150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego3_desiredTraj(simulationTrace):
    d_behind = 0
    d_in = -100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego3_safeTraj(simulationTrace):
    d_behind = 0
    d_in = -150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego4_desiredTraj(simulationTrace):
    d_behind = 100
    d_in = 100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego4_safeTraj(simulationTrace):
    d_behind = 100
    d_in = 200
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego5_desiredTraj(simulationTrace):
    d_behind = 100
    d_in = 75
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState
    
    
def ego5_safeTraj(simulationTrace):
    d_behind = 100
    d_in = 150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego6_desiredTraj(simulationTrace):
    d_behind = 100
    d_in = -100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego6_safeTraj(simulationTrace):
    d_behind = 100
    d_in = -150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState



def ego7_desiredTraj(simulationTrace):
    d_behind = 200
    d_in = 100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego7_safeTraj(simulationTrace):
    d_behind = 200
    d_in = 200
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego8_desiredTraj(simulationTrace):
    d_behind = 200
    d_in = 75
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState
    
    
def ego8_safeTraj(simulationTrace):
    d_behind = 200
    d_in = 150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego9_desiredTraj(simulationTrace):
    d_behind = 200
    d_in = -100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego9_safeTraj(simulationTrace):
    d_behind = 200
    d_in = -150
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState

def ego10_desiredTraj(simulationTrace):
    d_behind = 300
    d_in = 0
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState


def ego10_safeTraj(simulationTrace):
    d_behind = 300
    d_in = -100
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] -d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], lead_state[5]]
    return goalState



init_Trace = init_trace = {'agents':{'ego1':{'state_trace':[[0,-75,-75,0,pi/2,0,90]]}, 'leader':{'state_trace':[[0,100,0,0,0,0,100]]}}}


egoAgent1 = AircraftTrackingAgent("ego1",(ego1_desiredTraj,init_Trace, ego1_safeTraj),file_name=controllerFile)
egoAgent2 = AircraftTrackingAgent("ego2",(ego2_desiredTraj,init_Trace, ego2_safeTraj),file_name=controllerFile)
egoAgent3 = AircraftTrackingAgent("ego3",(ego3_desiredTraj,init_Trace, ego3_safeTraj),file_name=controllerFile)
egoAgent4 = AircraftTrackingAgent("ego4",(ego4_desiredTraj,init_Trace, ego4_safeTraj),file_name=controllerFile)
egoAgent5 = AircraftTrackingAgent("ego5",(ego5_desiredTraj,init_Trace, ego5_safeTraj),file_name=controllerFile)
egoAgent6 = AircraftTrackingAgent("ego6",(ego6_desiredTraj,init_Trace, ego6_safeTraj),file_name=controllerFile)
egoAgent7 = AircraftTrackingAgent("ego7",(ego7_desiredTraj,init_Trace, ego7_safeTraj),file_name=controllerFile)
egoAgent8 = AircraftTrackingAgent("ego8",(ego8_desiredTraj,init_Trace, ego8_safeTraj),file_name=controllerFile)
egoAgent9 = AircraftTrackingAgent("ego9",(ego9_desiredTraj,init_Trace, ego9_safeTraj),file_name=controllerFile)
egoAgent10 = AircraftTrackingAgent("ego10",(ego10_desiredTraj,init_Trace, ego10_safeTraj),file_name=controllerFile)


ego1Init = [-75,-75,0,0,0,90]
ego2Init = [-150,-150,0,0,0,90]
ego3Init = [0,-200,0,0,0,90]
ego4Init = [-275,-275,0,0,0,90]
ego5Init = [-350,-350,0,0,0,90]
ego6Init = [-100,-300,0,0,0,90]
ego7Init = [-475,-475,0,0,0,90]
ego8Init = [-550,-550,0,0,0,90]
ego9Init = [-200,-400,0,0,0,90]
ego10Init = [-300,-500,0,0,0,90]


otherAgent1 = AircraftAgent('leader', ([pi/18,0,0],),file_name=controllerFile)
otherInit = [0,0,0,0,0,100]

unsafe1 = relativeUnsafeBall("leader_ball", [0,0,0], 50, "leader")
unsafe2 = staticUnsafeRectangle("building",[470,570,0], 75, 75, 50)

ego1RTA = dubinsReachRTA(egoAgent1, [egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego2RTA = dubinsReachRTA(egoAgent2, [egoAgent1, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego3RTA = dubinsReachRTA(egoAgent3, [egoAgent1, egoAgent2, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego4RTA = dubinsReachRTA(egoAgent4, [egoAgent1, egoAgent2, egoAgent3, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego5RTA = dubinsReachRTA(egoAgent5, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego6RTA = dubinsReachRTA(egoAgent6, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego7RTA = dubinsReachRTA(egoAgent7, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent8, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego8RTA = dubinsReachRTA(egoAgent8, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent9, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego9RTA = dubinsReachRTA(egoAgent9, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent10, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego10RTA = dubinsReachRTA(egoAgent10, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, otherAgent1], [unsafe1, unsafe2], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])


rtas = [ego1RTA, ego2RTA, ego3RTA, ego4RTA, ego5RTA, ego6RTA, ego7RTA, ego8RTA, ego9RTA, ego10RTA, None]


agents = [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, egoAgent6, egoAgent7, egoAgent8, egoAgent9, egoAgent10, otherAgent1]
# rtas = [None, None]
modes = [AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.NORMAL]
initStates = [ego1Init, ego2Init, ego3Init, ego4Init, ego5Init, ego6Init, ego7Init, ego8Init, ego9Init, ego10Init, otherInit]

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
ego1RTA.eval.summary()
ego1RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego1RTA.eval.compute_agent_TTC(-1, 3, 4)

ego2RTA.eval.summary()
ego2RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego2RTA.eval.compute_agent_TTC(-1, 3, 4)


ego3RTA.eval.summary()
ego3RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego3RTA.eval.compute_agent_TTC(-1, 3, 4)

ego4RTA.eval.summary()
ego4RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego4RTA.eval.compute_agent_TTC(-1, 3, 4)

ego5RTA.eval.summary()
ego5RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego5RTA.eval.compute_agent_TTC(-1, 3, 4)


ego6RTA.eval.summary()
ego6RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego6RTA.eval.compute_agent_TTC(-1, 3, 4)


ego7RTA.eval.summary()
ego7RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego7RTA.eval.compute_agent_TTC(-1, 3, 4)

ego8RTA.eval.summary()
ego8RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego8RTA.eval.compute_agent_TTC(-1, 3, 4)


ego9RTA.eval.summary()
ego9RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego9RTA.eval.compute_agent_TTC(-1, 3, 4)


ego10RTA.eval.summary()
ego10RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego10RTA.eval.compute_agent_TTC(-1, 3, 4)
print('Evaluation time: ', time.time() - start_time)

untrusted_usage, safety_usage, num_switches1 = ego1RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches2 = ego2RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches3 = ego3RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches4 = ego4RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches5 = ego5RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches6 = ego6RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches7 = ego7RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches8 = ego8RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches9 = ego9RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches10 = ego10RTA.eval.controllerUsage()

averageRT1 = sum(ego1RTA.eval.computation_times)/(len(ego1RTA.eval.computation_times))
averageRT2 = sum(ego2RTA.eval.computation_times)/(len(ego2RTA.eval.computation_times))
averageRT3 = sum(ego3RTA.eval.computation_times)/(len(ego3RTA.eval.computation_times))
averageRT4 = sum(ego4RTA.eval.computation_times)/(len(ego4RTA.eval.computation_times))
averageRT5 = sum(ego5RTA.eval.computation_times)/(len(ego5RTA.eval.computation_times))
averageRT6 = sum(ego6RTA.eval.computation_times)/(len(ego6RTA.eval.computation_times))
averageRT7 = sum(ego7RTA.eval.computation_times)/(len(ego7RTA.eval.computation_times))
averageRT8 = sum(ego8RTA.eval.computation_times)/(len(ego8RTA.eval.computation_times))
averageRT9 = sum(ego9RTA.eval.computation_times)/(len(ego9RTA.eval.computation_times))
averageRT10 = sum(ego10RTA.eval.computation_times)/(len(ego10RTA.eval.computation_times))

print('Average Computation time: ', (averageRT1+averageRT2+averageRT3+averageRT4+averageRT5+averageRT6+averageRT7+averageRT8+averageRT9+averageRT10)/10)
print('Num switches: ', num_switches1+num_switches2+num_switches3+num_switches4+num_switches5+num_switches6+num_switches7+num_switches8+num_switches9+num_switches10)
print('Execution time: ', exec_time)