import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/acc/five_acc_sim.py', '')
controllerFile = curr_path.replace('five_acc_sim.py', 'acc_utils/cc_controller.py')
sys.path.append(baseDirPath)

import time

from utils.simpleSim import simpleSim
from utils.unsafeSets import relativeUnsafeBall

from acc_utils.cc_agent import *
from acc_utils.cc_controller import *
from acc_utils.accRTA import accSimRTA, accReachRTA

from demos.demo_rta.simRTA import *

# Ego:
def ego1_desiredTraj(simulationTrace):
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 10, lead_state[1]]
    return goalState

def ego2_desiredTraj(simulationTrace):
    d_behind = 20
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind, lead_state[1]]
    return goalState

def ego3_desiredTraj(simulationTrace):
    d_behind = 30
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind, lead_state[1]]
    return goalState

def ego4_desiredTraj(simulationTrace):
    d_behind = 40
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind, lead_state[1]]
    return goalState

def ego5_desiredTraj(simulationTrace):
    d_behind = 50
    lead_state = simulationTrace['agents']['other1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - d_behind, lead_state[1]]
    return goalState










egoAgent1 = ccAgent("ego1",file_name=controllerFile)
egoAgent1.follower = True
egoAgent1.desired_traj = ego1_desiredTraj
ego1_init = [-10, 1]

egoAgent2 = ccAgent("ego2",file_name=controllerFile)
egoAgent2.follower = True
egoAgent2.desired_traj = ego2_desiredTraj
ego2_init = [-20, 1]

egoAgent3 = ccAgent("ego3",file_name=controllerFile)
egoAgent3.follower = True
egoAgent3.desired_traj = ego3_desiredTraj
ego3_init = [-30, 1]

egoAgent4 = ccAgent("ego4",file_name=controllerFile)
egoAgent4.follower = True
egoAgent4.desired_traj = ego4_desiredTraj
ego4_init = [-40, 1]

egoAgent5 = ccAgent("ego5",file_name=controllerFile)
egoAgent5.follower = True
egoAgent5.desired_traj = ego5_desiredTraj
ego5_init = [-50, 1]

# Lead:
otherAgent1 = ccAgent("other1",file_name=controllerFile)
other_init = [5, 1]

# Agents:
agents = [egoAgent1, egoAgent2, egoAgent3, egoAgent4, egoAgent5, otherAgent1]
modes = [ccMode.UNTRUSTED, ccMode.UNTRUSTED, ccMode.UNTRUSTED, ccMode.UNTRUSTED, ccMode.UNTRUSTED, ccMode.NORMAL]
initStates = [ego1_init, ego2_init, ego3_init, ego4_init, ego5_init, other_init]

# Unsafe sets:
unsafe1 = relativeUnsafeBall("unsafe1", [5], 7, "other1")

init_trace = {'agents':{'ego1':{'state_trace':[[0]+initStates[0]], 'mode_trace':[modes[0]], 'ego2':{'state_trace':[[0]+initStates[1]], 'mode_trace':[modes[0]]}, 'other1':{'state_trace':[[0]+initStates[2]], 'mode_trace':[modes[1]]}}, 'unsafe':{'unsafe1':{'set_type':'ball', 'state_trace':[[0,([5],7)]]}}}}

# Set up RTA:
ego1RTA = accSimRTA(egoAgent1, [egoAgent2, egoAgent3, egoAgent4, egoAgent5, otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL])
ego2RTA = accSimRTA(egoAgent2, [egoAgent1, egoAgent3, egoAgent4, egoAgent5, otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL])
ego3RTA = accSimRTA(egoAgent3, [egoAgent1, egoAgent2, egoAgent4, egoAgent5, otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL])
ego4RTA = accSimRTA(egoAgent4, [egoAgent1, egoAgent2, egoAgent3, egoAgent5, otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL])
ego5RTA = accSimRTA(egoAgent5, [egoAgent1, egoAgent2, egoAgent3, egoAgent4, otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL, ccMode.NORMAL])

RTAs = [ego1RTA, ego2RTA, ego3RTA, ego4RTA, ego5RTA, None]


accSim = simpleSim()
accSim.setSimType(vis=False, plotType="2D", simType="1D")
accSim.setTimeParams(dt=0.05, T=10)
accSim.addAgents(agents=agents, modes=modes, RTAs=RTAs, initStates=initStates)
accSim.addUnsafeSets(unsafe_sets = [unsafe1])
accSim.chooseAxesLimits("relative", "other1", [-30,5], [-5,5], [-5,5])

start_time = time.time()
simulationTrace = accSim.runSim()
exec_time = time.time() - start_time

start_time = time.time()
ego1RTA.eval.summary(False)
ego1RTA.eval.compute_unsafe_TTC(1)
ego1RTA.eval.compute_agent_TTC(1)

ego2RTA.eval.summary(False)
ego2RTA.eval.compute_unsafe_TTC(1)
ego2RTA.eval.compute_agent_TTC(1)

ego3RTA.eval.summary(False)
ego3RTA.eval.compute_unsafe_TTC(1)
ego3RTA.eval.compute_agent_TTC(1)

ego4RTA.eval.summary(False)
ego4RTA.eval.compute_unsafe_TTC(1)
ego4RTA.eval.compute_agent_TTC(1)

ego5RTA.eval.summary(False)
ego5RTA.eval.compute_unsafe_TTC(1)
ego5RTA.eval.compute_agent_TTC(1)
print('Evaluation time: ', time.time() -start_time)

untrusted_usage, safety_usage, num_switches1 = ego1RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches2 = ego2RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches3 = ego3RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches4 = ego4RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches5 = ego5RTA.eval.controllerUsage()

averageRT1 = sum(ego1RTA.eval.computation_times)/(len(ego1RTA.eval.computation_times))
averageRT2 = sum(ego2RTA.eval.computation_times)/(len(ego2RTA.eval.computation_times))
averageRT3 = sum(ego3RTA.eval.computation_times)/(len(ego3RTA.eval.computation_times))
averageRT4 = sum(ego4RTA.eval.computation_times)/(len(ego4RTA.eval.computation_times))
averageRT5 = sum(ego5RTA.eval.computation_times)/(len(ego5RTA.eval.computation_times))

print('Average Computation time: ', (averageRT1+averageRT2+averageRT3+averageRT4+averageRT5)/5)
print('Num switches: ', num_switches1+num_switches2+num_switches3+num_switches4+num_switches5)
print('Execution time: ', exec_time)


