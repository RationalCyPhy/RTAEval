import os, sys
curr_path = os.path.realpath(__file__)
baseDirPath = curr_path.replace('demos/demo_rta/simRTA.py', '')
sys.path.append(baseDirPath)

import copy

from utils.baseRTA import baseRTA

class simRTA(baseRTA):
    def __init__(self, egoAgent, agents, unsafeSets, egoModes, otherModes, dt = 0.1, T = 0.5):
        super().__init__()
        self.egoAgent = copy.deepcopy(egoAgent)
        self.agents = copy.deepcopy(agents)
        self.unsafeSets = copy.deepcopy(unsafeSets)
        self.dt = 0.1
        self.T = 0.5
        self.egoModes = egoModes
        self.otherModes = otherModes

    def simulate_forward(self, simulationTrace):
        t = 0
        egoInit = simulationTrace['agents'][self.egoAgent.id]['state_trace'][-1][1:]

        predicted_traj = {'agents':{}, 'unsafe':{}}
        predicted_traj['agents'][self.egoAgent.id] = {'state_trace':[[t]+egoInit]}

        for agent in self.agents:
            agentInit = simulationTrace['agents'][agent.id]['state_trace'][-1][1:]
            predicted_traj['agents'][agent.id] = {'state_trace':[[t]+agentInit]}

        for unsafeSet in self.unsafeSets:
            unsafeSetInit = simulationTrace['unsafe'][unsafeSet.id]['state_trace'][-1][1:]
            predicted_traj['unsafe'][unsafeSet.id] = {'state_trace':[[t] + unsafeSetInit]}

        while t < self.T:
            t += self.dt

            for agentNum in range(len(self.agents)):
                agent = self.agents[agentNum]
                agentMode = self.otherModes[agentNum]
                agentInit = predicted_traj['agents'][agent.id]['state_trace'][-1][1:]
                agentInit = agent.step([agentMode], agentInit, self.dt, predicted_traj)
                predicted_traj['agents'][agent.id]['state_trace'].append([t]+agentInit)
                

            for unsafeSet in self.unsafeSets:
                new_set_def = [t, unsafeSet.update_def(predicted_traj)]
                predicted_traj['unsafe'][unsafeSet.id]['state_trace'].append(new_set_def)

            egoInit = self.egoAgent.step([self.egoModes.UNTRUSTED], egoInit, self.dt, predicted_traj)
            predicted_traj['agents'][self.egoAgent.id]['state_trace'].append([t]+egoInit)

        return predicted_traj

    def RTALogic(self, simulationTrace: dict):
        return self.egoModes.UNTRUSTED
