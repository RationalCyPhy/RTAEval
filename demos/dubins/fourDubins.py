import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/dubins/fourDubins.py', '')
sys.path.append(baseDirPath)

import time

from utils.simpleSim import simpleSim
from utils.unsafeSets import *

from dubins_utils.agent_aircraft import *
from dubins_utils.controller_aircraft import *
from dubins_utils.dubinsRTA import dubinsSimRTA, dubinsReachRTA

controllerFile = curr_path.replace('fourDubins.py', 'dubins_utils/controller_aircraft.py')

def ego1_desiredTraj(simulationTrace):
    d_behind = 250
    d_in = 100
    vel_ref = 480
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego1_safeTraj(simulationTrace):
    d_behind = 250
    d_in = 300
    vel_ref = 440
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego2_desiredTraj(simulationTrace):
    d_behind = 250
    d_in = -100
    vel_ref = 520
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego2_safeTraj(simulationTrace):
    d_behind = 250
    d_in = -300
    vel_ref = 560
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego3_desiredTraj(simulationTrace):
    d_behind = 500
    d_in = 200
    vel_ref = 460
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego3_safeTraj(simulationTrace):
    d_behind = 500
    d_in = 400
    vel_ref = 420
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego4_desiredTraj(simulationTrace):
    d_behind = 500
    d_in = -200
    vel_ref = 540
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState

def ego4_safeTraj(simulationTrace):
    d_behind = 500
    d_in = -400
    vel_ref = 580
    lead_state = simulationTrace['agents']['leader']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind*cos(lead_state[3]) - d_in*sin(lead_state[3]), lead_state[1] - d_behind*sin(lead_state[3]) + d_in*cos(lead_state[3]), lead_state[2], lead_state[3], lead_state[4], vel_ref]
    return goalState


init_Trace = init_trace = {'agents':{'ego1':{'state_trace':[[0,-75,-75,0,pi/2,0,90]]}, 'leader':{'state_trace':[[0,100,0,0,0,0,100]]}}}


egoAgent1 = AircraftTrackingAgent("ego1",(ego1_desiredTraj,init_Trace, ego1_safeTraj),file_name=controllerFile)
egoAgent1.K1 = [10, 1, 100, 1]
egoAgent1.K2 = [10, 10]
egoAgent2 = AircraftTrackingAgent("ego2",(ego2_desiredTraj,init_Trace, ego2_safeTraj),file_name=controllerFile)
egoAgent2.K1 = [10, 1, 100, 1]
egoAgent2.K2 = [10, 10]
egoAgent3 = AircraftTrackingAgent("ego3",(ego3_desiredTraj,init_Trace, ego3_safeTraj),file_name=controllerFile)
egoAgent3.K1 = [10, 1, 100, 1]
egoAgent3.K2 = [10, 10]
egoAgent4 = AircraftTrackingAgent("ego4",(ego4_desiredTraj,init_Trace, ego4_safeTraj),file_name=controllerFile)
egoAgent4.K1 = [10, 1, 100, 1]
egoAgent4.K2 = [10, 10]

ego1Init = [900,-250,0,0,0,480]
ego2Init = [1100,-250,0,0,0,520]
ego3Init = [800,-500,0,0,0,460]
ego4Init = [1200,-500,0,0,0,540]


otherAgent1 = AircraftAgent('leader', ([0.2,0,0],),file_name=controllerFile)
otherInit = [1000,0,0,pi/2,0,500]

unsafe1 = relativeUnsafeBall("leader_ball", [0,0,0], 50, "leader")
building = staticUnsafeRectangle("building",[-3880,0,0], 100, 100, 100)

ego1RTA = dubinsReachRTA(egoAgent1, [egoAgent2, egoAgent3, egoAgent4, otherAgent1], [unsafe1, building], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego2RTA = dubinsReachRTA(egoAgent2, [egoAgent1, egoAgent3, egoAgent4, otherAgent1], [unsafe1, building], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego3RTA = dubinsReachRTA(egoAgent3, [egoAgent1, egoAgent2, egoAgent4, otherAgent1], [unsafe1, building], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])
ego4RTA = dubinsReachRTA(egoAgent4, [egoAgent1, egoAgent2, egoAgent3, otherAgent1], [unsafe1, building], AircraftMode, [AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL, AircraftMode.NORMAL])


agents = [otherAgent1, egoAgent1, egoAgent2, egoAgent3, egoAgent4]
rtas = [None, ego1RTA, ego2RTA, ego3RTA, ego4RTA]
modes = [AircraftMode.NORMAL, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED, AircraftMode.UNTRUSTED]
initStates = [otherInit, ego1Init, ego2Init, ego3Init, ego4Init]

dubinsSim = simpleSim()
dubinsSim.setSimType(vis=False, plotType="2D", simType="3D")
dubinsSim.setTimeParams(dt=0.1, T=40)
dubinsSim.addAgents(agents=agents, RTAs=rtas, modes=modes, initStates=initStates)
dubinsSim.addUnsafeSets(unsafe_sets = [unsafe1, building])
dubinsSim.chooseAxesLimits("relative", "leader", [-1000,1000], [-1000,1000], [-100,100])

start_time = time.time()
simulationTrace = dubinsSim.runSim()
print('Execution time: ', time.time() - start_time)

print('EGO 1')
ego1RTA.eval.summary()
ego1RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego1RTA.eval.compute_agent_TTC(-1, 3, 4)
print('####################')
print('####################')
print('####################')

print('EGO 2')
ego2RTA.eval.summary()
ego2RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego2RTA.eval.compute_agent_TTC(-1, 3, 4)
print('####################')
print('####################')
print('####################')

print('EGO 3')
ego3RTA.eval.summary()
ego3RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego3RTA.eval.compute_agent_TTC(-1, 3, 4)
print('####################')
print('####################')
print('####################')

print('EGO 4')
ego4RTA.eval.summary()
ego4RTA.eval.compute_unsafe_TTC(-1, 3, 4)
ego4RTA.eval.compute_agent_TTC(-1, 3, 4)
print('####################')
print('####################')
print('####################')

