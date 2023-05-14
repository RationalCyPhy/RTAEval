import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/acc/two_acc_sim.py', '')
controllerFile = curr_path.replace('two_acc_sim.py', 'acc_utils/cc_controller.py')
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
    lead_state = simulationTrace['agents']['ego1']['state_trace'][-1][1:]
    goalState = [lead_state[0] - 10, lead_state[1]]
    return goalState

egoAgent1 = ccAgent("ego1",file_name=controllerFile)
egoAgent1.follower = True
egoAgent1.desired_traj = ego1_desiredTraj

egoAgent2 = ccAgent("ego2",file_name=controllerFile)
egoAgent2.follower = True
egoAgent2.desired_traj = ego2_desiredTraj

# Lead:
otherAgent1 = ccAgent("other1",file_name=controllerFile)

# Agents:
agents = [egoAgent1, egoAgent2, otherAgent1]
modes = [ccMode.UNTRUSTED, ccMode.UNTRUSTED, ccMode.NORMAL]
initStates = [[-10,1], [-20,1], [5,1]]

# Unsafe sets:
unsafe1 = relativeUnsafeBall("unsafe1", [5], 7, "other1")

init_trace = {'agents':{'ego1':{'state_trace':[[0]+initStates[0]], 'mode_trace':[modes[0]], 'ego2':{'state_trace':[[0]+initStates[1]], 'mode_trace':[modes[0]]}, 'other1':{'state_trace':[[0]+initStates[2]], 'mode_trace':[modes[1]]}}, 'unsafe':{'unsafe1':{'set_type':'ball', 'state_trace':[[0,([5],7)]]}}}}

# Set up RTA:
ego1RTA = accSimRTA(egoAgent1, [egoAgent2,otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL])
ego2RTA = accSimRTA(egoAgent2, [egoAgent1,otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL, ccMode.NORMAL])

RTAs = [ego1RTA, ego2RTA, None]


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
print('Evaluation time: ', time.time() -start_time)

untrusted_usage, safety_usage, num_switches1 = ego1RTA.eval.controllerUsage()
untrusted_usage, safety_usage, num_switches2 = ego2RTA.eval.controllerUsage()

averageRT1 = sum(ego1RTA.eval.computation_times)/(len(ego1RTA.eval.computation_times))
averageRT2 = sum(ego2RTA.eval.computation_times)/(len(ego2RTA.eval.computation_times))

print('Average Computation time: ', (averageRT1+averageRT2)/2)
print('Num switches: ', num_switches1+num_switches2)
print('Execution time: ', exec_time)


