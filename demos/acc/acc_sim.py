import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/acc/acc_sim.py', '')
controllerFile = curr_path.replace('acc_sim.py', 'acc_utils/cc_controller.py')
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

egoAgent1 = ccAgent("ego1",file_name=controllerFile)
egoAgent1.follower = True
egoAgent1.desired_traj = ego1_desiredTraj

# Lead:
otherAgent1 = ccAgent("other1",file_name=controllerFile)

# Agents:
agents = [egoAgent1, otherAgent1]
modes = [ccMode.UNTRUSTED, ccMode.NORMAL]
initStates = [[-10,1], [5,1]]

# Unsafe sets:
unsafe1 = relativeUnsafeBall("unsafe1", [5], 7, "other1")

init_trace = {'agents':{'ego1':{'state_trace':[[0,0,1]], 'mode_trace':[modes[0]]}, 'other1':{'state_trace':[[0,5,1]], 'mode_trace':[modes[1]]}}, 'unsafe':{'unsafe1':{'set_type':'ball', 'state_trace':[[0,([5],7)]]}}}

# Set up RTA:
egoRTA = accSimRTA(egoAgent1, [otherAgent1], [unsafe1], ccMode, [ccMode.NORMAL])
RTAs = [egoRTA, None]


accSim = simpleSim()
accSim.setSimType(vis=False, plotType="2D", simType="1D")
accSim.setTimeParams(dt=0.05, T=10)
accSim.addAgents(agents=agents, modes=modes, RTAs=RTAs, initStates=initStates)
accSim.addUnsafeSets(unsafe_sets = [unsafe1])
accSim.chooseAxesLimits("relative", "other1", [-20,5], [-5,5], [-5,5])

start_time = time.time()
simulationTrace = accSim.runSim()
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

