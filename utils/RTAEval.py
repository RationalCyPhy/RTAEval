
import matplotlib.pyplot as plt
import matplotlib
from math import ceil, sqrt, sin, cos
import numpy as np
import cvxpy as cp

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

matplotlib.rc('font', **font)

class RTAEval:
    def __init__(self):
        self.simulation_trace = None
        self.computation_times = []
        self.egoID = None
        self.dists = None
        self.workspaceDims = 2

    def collect_trace(self, curr_state):
        # This is where collecting the trace will happen
        if self.simulation_trace == None:
            self.simulation_trace = {'agents':{}, 'unsafe':{}}
            
            id_list = list(curr_state['agents'].keys())
            for id in id_list:
                self.simulation_trace['agents'][id] = {'state_trace':[], 'mode_trace':[]}
                self.simulation_trace['agents'][id]['state_trace'].append(curr_state['agents'][id]['state_trace'][-1])
                self.simulation_trace['agents'][id]['mode_trace'].append(curr_state['agents'][id]['mode_trace'][-1])

            unsafeIDs = list(curr_state['unsafe'].keys())
            for unsafeKey in unsafeIDs:
                self.simulation_trace['unsafe'][unsafeKey] = {'set_type':curr_state['unsafe'][unsafeKey]['set_type'], 'state_trace':[]}
                self.simulation_trace['unsafe'][unsafeKey]['state_trace'].append(curr_state['unsafe'][unsafeKey]['state_trace'][-1])

        else:
            id_list = list(self.simulation_trace['agents'].keys())
            for id in id_list:
                self.simulation_trace['agents'][id]['state_trace'].append(curr_state['agents'][id]['state_trace'][-1])
                self.simulation_trace['agents'][id]['mode_trace'].append(curr_state['agents'][id]['mode_trace'][-1])

            unsafeIDs = list(self.simulation_trace['unsafe'].keys())
            for unsafeKey in unsafeIDs:
                self.simulation_trace['unsafe'][unsafeKey]['state_trace'].append(curr_state['unsafe'][unsafeKey]['state_trace'][-1])

        return None

    def collect_computation_time(self, running_time):
        self.computation_times.append(running_time)
        return None
    
    def trace_at_max_RT(self):
        # State of simulation at max comp time
        maxRT = max(self.computation_times)
        max_idx = self.computation_times.index(maxRT)

        curr_trace = {'agents':{}, 'unsafe':{}}

        id_list = list(self.simulation_trace['agents'].keys())
        for id in id_list:
            curr_trace['agents'][id] = {'state_trace':[], 'mode_trace':[]}

            curr_trace['agents'][id]['state_trace'] = [self.simulation_trace['agents'][id]['state_trace'][max_idx]]
            curr_trace['agents'][id]['mode_trace'] = [self.simulation_trace['agents'][id]['mode_trace'][max_idx]]

        unsafeIDs = list(self.simulation_trace['unsafe'].keys())
        for id in unsafeIDs:
            curr_trace['unsafe'][id] = {'set_type':[], 'state_trace':[]}
            
            curr_trace['unsafe'][id]['set_type'] = self.simulation_trace['unsafe'][id]['set_type']
            curr_trace['unsafe'][id]['state_trace'] = [self.simulation_trace['unsafe'][id]['state_trace'][max_idx]]

        return curr_trace
    
    def trace_at_min_RT(self):
        # State of simulation at min comp time
        minRT = min(self.computation_times)
        min_idx = self.computation_times.index(minRT)

        curr_trace = {'agents':{}, 'unsafe':{}}

        id_list = list(self.simulation_trace['agents'].keys())
        for id in id_list:
            curr_trace['agents'][id] = {'state_trace':[], 'mode_trace':[]}

            curr_trace['agents'][id]['state_trace'] = [self.simulation_trace['agents'][id]['state_trace'][min_idx]]
            curr_trace['agents'][id]['mode_trace'] = [self.simulation_trace['agents'][id]['mode_trace'][min_idx]]

        unsafeIDs = list(self.simulation_trace['unsafe'].keys())
        for id in unsafeIDs:
            curr_trace['unsafe'][id] = {'set_type':[], 'state_trace':[]}
            
            curr_trace['unsafe'][id]['set_type'] = self.simulation_trace['unsafe'][id]['set_type']
            curr_trace['unsafe'][id]['state_trace'] = [self.simulation_trace['unsafe'][id]['state_trace'][min_idx]]

        return curr_trace

    def controllerUsage(self, showPlot = False):
        mode_trace = self.simulation_trace['agents'][self.egoID]['mode_trace']
        dt = self.simulation_trace['agents'][self.egoID]['state_trace'][-1][0] - self.simulation_trace['agents'][self.egoID]['state_trace'][-2][0]

        untrusted_modes = []
        safety_modes = []
        times = []
        switch_idxs = []
        curr_mode = 2
        for i in range(len(mode_trace)):
            mode = mode_trace[i]
            times.append(i*dt)

            if mode.value != curr_mode:
                switch_idxs.append(i)

            if mode.value ==2: #UNTRUSTED
                untrusted_modes.append(1)
                safety_modes.append(0)
                curr_mode = 2
            else: #SAFETY
                untrusted_modes.append(0)
                safety_modes.append(1)
                curr_mode = 3

        num_switches = len(switch_idxs)
        untrusted_usage = sum(untrusted_modes)/(len(untrusted_modes))
        safety_usage = sum(safety_modes)/(len(safety_modes))

        if showPlot:
            plt.figure()

            if len(switch_idxs) == 0:
                plt.plot(times, untrusted_modes, 'orange', linewidth=10, label='Untrusted')
            else:
                init_idx = 0
                for switch in switch_idxs:
                    if untrusted_modes[switch - 1] == 1:
                        plt.plot(times[init_idx:switch], untrusted_modes[init_idx:switch], 'orange', linewidth=10, label='Untrusted')
                    else:
                        plt.plot(times[init_idx:switch], safety_modes[init_idx:switch], 'blue', linewidth=10, label='Safety')
                    init_idx = switch

                if untrusted_modes[-1] == 1:
                    plt.plot(times[switch:len(times)], untrusted_modes[switch:len(times)], 'orange', linewidth=10, label='Untrusted')
                else:
                    plt.plot(times[switch:len(times)], safety_modes[switch:len(times)], 'blue', linewidth=10, label='Safety')

            
            plt.title('Untrusted vs. Safety Controller Usage')
            plt.xlabel('Time')
            plt.yticks([])
            plt.ylim([0,2])
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys())
            plt.grid()

        return untrusted_usage, safety_usage, num_switches

    def compute_dists(self, showPlot = False):
        self.dists = {}

        ego_trace = self.simulation_trace['agents'][self.egoID]['state_trace']
        for unsafeID in list(self.simulation_trace['unsafe'].keys()):
            self.dists[unsafeID] = []
            if self.simulation_trace['unsafe'][unsafeID]['set_type'] == 'ball':
                for i in range(len(ego_trace)):
                    setDef = self.simulation_trace['unsafe'][unsafeID]['state_trace'][i][1:][0]
                    r = setDef[1]
                    center = setDef[0]
                    self.dists[unsafeID].append(computeBallDist(ego_trace[i][1:], center, r))

            elif self.simulation_trace['unsafe'][unsafeID]['set_type'] == 'rectangle':
                for i in range(len(ego_trace)):
                    setDef = self.simulation_trace['unsafe'][unsafeID]['state_trace'][i][1:][0]
                    self.dists[unsafeID].append(computeRectDist(ego_trace[i][1:], setDef))
            
            elif self.simulation_trace['unsafe'][unsafeID]['set_type'] == 'polytope':
                if self.workspaceDims == 1:
                    raise Exception('Polytope calculations only supported in 2 or 3 dimensional workspaces!')
                elif self.workspaceDims == 2:
                    for i in range(len(ego_trace)):
                        setDef = self.simulation_trace['unsafe'][unsafeID]['state_trace'][i][1:][0]
                        A, b = setDef

                        x = cp.Variable(1)
                        y = cp.Variable(1)

                        constraints = []
                        for row in range(len(b)):
                            constraints.append(A[row][0]*x + A[row][1]*y  <= b[row])

                        curr_state = ego_trace[i][1:]
                        
                        objective = cp.Minimize((curr_state[0] - x)**2 + (curr_state[1] - y)**2)
                        prob = cp.Problem(objective, constraints)
                        result = prob.solve()
                        self.dists[unsafeID].append(sqrt(result))
                        
                elif self.workspaceDims == 3:
                    for i in range(len(ego_trace)):
                        setDef = self.simulation_trace['unsafe'][unsafeID]['state_trace'][i][1:][0]
                        A, b = setDef

                        x = cp.Variable(1)
                        y = cp.Variable(1)
                        z = cp.Variable(1)

                        constraints = []
                        for row in range(len(b)):
                            constraints.append(A[row][0]*x + A[row][1]*y + A[row][2]*z <= b[row])

                        curr_state = ego_trace[i][1:]
                        
                        objective = cp.Minimize((curr_state[0] - x)**2 + (curr_state[1] - y)**2 + (curr_state[2] - z)**2)
                        prob = cp.Problem(objective, constraints)
                        result = prob.solve()
                        self.dists[unsafeID].append(sqrt(result))

        minDists = [min(self.dists[unsafeID]) for unsafeID in list(self.dists.keys())]
        maxDists = [max(self.dists[unsafeID]) for unsafeID in list(self.dists.keys())]
        
        avgDists = {}
        for unsafeID in list(self.dists.keys()):
            avgDists[unsafeID] = sum(self.dists[unsafeID])/len(self.dists[unsafeID])

        if showPlot:
            plt.figure()
            dt = self.simulation_trace['agents'][self.egoID]['state_trace'][-1][0] - self.simulation_trace['agents'][self.egoID]['state_trace'][-2][0]

            times = [i*dt for i in range(len(ego_trace))]
            for unsafeID in list(self.dists.keys()):
                dists = self.dists[unsafeID]
                plt.plot(times, dists, label=unsafeID)
            
            plt.title('Distance from unsafe sets')
            plt.xlabel('Time')
            plt.ylabel('Distance')
            plt.legend()
            plt.grid()
        
        return min(minDists), max(maxDists), avgDists

    def compute_agent_dists(self, showPlot = False):
        self.agent_dists = {}

        ego_trace = self.simulation_trace['agents'][self.egoID]['state_trace']
        for agentID in list(self.simulation_trace['agents'].keys()):
            if agentID != self.egoID:
                self.agent_dists[agentID] = []

                agent_trace = self.simulation_trace['agents'][agentID]['state_trace']
                for i in range(len(ego_trace)):
                    agent_state = agent_trace[i][1:]
                    ego_state = ego_trace[i][1:]

                    x_dist = agent_state[0] - ego_state[0]
                    y_dist = 0
                    z_dist = 0
                    if self.workspaceDims > 1:
                        y_dist = agent_state[1] - ego_state[1]
                    elif self.workspaceDims > 2:
                        z_dist = agent_state[1] - ego_state[1]


                    self.agent_dists[agentID].append(sqrt(x_dist**2 + y_dist**2 + z_dist**2))


        minDists = [min(self.agent_dists[agentID]) for agentID in list(self.agent_dists.keys())]
        maxDists = [max(self.agent_dists[agentID]) for agentID in list(self.agent_dists.keys())]
        
        avgDists = {}
        for agentID in list(self.agent_dists.keys()):
            avgDists[agentID] = sum(self.agent_dists[agentID])/len(self.agent_dists[agentID])

        if showPlot:
            plt.figure()
            dt = self.simulation_trace['agents'][self.egoID]['state_trace'][-1][0] - self.simulation_trace['agents'][self.egoID]['state_trace'][-2][0]

            times = [i*dt for i in range(len(ego_trace))]
            for agentID in list(self.agent_dists.keys()):
                dists = self.agent_dists[agentID]
                plt.plot(times, dists, label=agentID)
            
            plt.title('Distance from other agents')
            plt.xlabel('Time')
            plt.ylabel('Distance')
            plt.legend()
            plt.grid()
        
        return min(minDists), max(maxDists), avgDists

    def compute_unsafe_TTC(self, vel_idx, heading_idx = None, pitch_idx = None):
        if self.workspaceDims > 1 and heading_idx == None:
            raise Exception('Must provide heading index for 2D or 3D scenario')
        if self.workspaceDims > 2 and pitch_idx == None:
            raise Exception('Must provide pitch index for 3D scenario')

        ego_trace = self.simulation_trace['agents'][self.egoID]['state_trace']
        
        self.unsafe_TTC = {}
        for unsafeID in list(self.simulation_trace['unsafe'].keys()):
            self.unsafe_TTC[unsafeID] = []
            unsafe_trace = self.simulation_trace['unsafe'][unsafeID]['state_trace']
            unsafe_type = self.simulation_trace['unsafe'][unsafeID]['set_type']

            for i in range(len(ego_trace)):
                ego_state = ego_trace[i][1:]
                unsafe_def = unsafe_trace[i][1:][0]

                if self.workspaceDims == 1:
                    ego_vel_vec = np.array([ego_state[vel_idx], 0, 0])
                elif self.workspaceDims == 2:
                    ego_vel_vec = np.array([cos(ego_state[heading_idx]), sin(ego_state[heading_idx]), 0])*ego_state[vel_idx]
                elif self.workspaceDims == 3:
                    ego_vel_vec = np.array([cos(ego_state[heading_idx])*cos(ego_state[pitch_idx]), sin(ego_state[heading_idx])*cos(ego_state[pitch_idx]), sin(ego_state[pitch_idx])])*ego_state[vel_idx]
                
                if unsafe_type == 'ball':
                    r = unsafe_def[1]
                    center = unsafe_def[0]

                    if self.workspaceDims == 1:
                        ego_to_center = np.array([center[0] - ego_state[0], 0, 0])
                    elif self.workspaceDims == 2:
                        ego_to_center = np.array([center[0] - ego_state[0], center[1] - ego_state[1], 0])
                    elif self.workspaceDims == 3:
                        ego_to_center = np.array([center[0] - ego_state[0], center[1] - ego_state[1], center[2] - ego_state[2]])
                    
                    dist_to_unsafe = computeBallDist(ego_trace[i][1:], center, r)

                elif unsafe_type == 'rectangle':
                    center = unsafe_def[0]

                    if self.workspaceDims == 1:
                        ego_to_center = np.array([center[0] - ego_state[0], 0, 0])
                    elif self.workspaceDims == 2:
                        ego_to_center = np.array([center[0] - ego_state[0], center[1] - ego_state[1], 0])
                    elif self.workspaceDims == 3:
                        ego_to_center = np.array([center[0] - ego_state[0], center[1] - ego_state[1], center[2] - ego_state[2]])
                    
                    dist_to_unsafe = computeRectDist(ego_trace[i][1:], unsafe_def)

                elif self.simulation_trace['unsafe'][unsafeID]['set_type'] == 'polytope':
                    if self.workspaceDims == 1:
                        raise Exception('Polytope caluclations only supported in 2 or 3 dimensional workspaces!')
                    elif self.workspaceDims == 2:
                        A, b = unsafe_def

                        x = cp.Variable(1)
                        y = cp.Variable(1)

                        constraints = []
                        for row in range(len(b)):
                            constraints.append(A[row][0]*x + A[row][1]*y <= b[row])

                        curr_state = ego_trace[i][1:]
                        
                        objective = cp.Minimize((curr_state[0] - x)**2 + (curr_state[1] - y)**2)
                        prob = cp.Problem(objective, constraints)
                        result = prob.solve()

                        ego_to_center = np.array([x.value - ego_state[0], y.value - ego_state[1]])


                    elif self.workspaceDims == 3:
                        A, b = unsafe_def

                        x = cp.Variable(1)
                        y = cp.Variable(1)
                        z = cp.Variable(1)

                        constraints = []
                        for row in range(len(b)):
                            constraints.append(A[row][0]*x + A[row][1]*y + A[row][2]*z <= b[row])

                        curr_state = ego_trace[i][1:]
                        
                        objective = cp.Minimize((curr_state[0] - x)**2 + (curr_state[1] - y)**2 + (curr_state[2] - z)**2)
                        prob = cp.Problem(objective, constraints)
                        result = prob.solve()

                        ego_to_center = np.array([x.value - ego_state[0], y.value - ego_state[1], z.value - ego_state[2]])


                vel_to_center = (np.dot(ego_vel_vec, ego_to_center)/(np.linalg.norm(ego_to_center)**2))*ego_to_center
                time_to_collision = dist_to_unsafe/np.linalg.norm(vel_to_center)
                
                self.unsafe_TTC[unsafeID].append(time_to_collision)
        
        maxUnsafeTTC = {}
        minUnsafeTTC = {}
        avgUnsafeTTC = {}
        for unsafeID in list(self.unsafe_TTC.keys()):
            avgUnsafeTTC[unsafeID] = sum(self.unsafe_TTC[unsafeID])/len(self.unsafe_TTC[unsafeID])
            minUnsafeTTC[unsafeID] = min(self.unsafe_TTC[unsafeID])
            maxUnsafeTTC[unsafeID] = max(self.unsafe_TTC[unsafeID])

        print('Unsafe set time to collision metrics:')
        print('Minimum time to collision: ', minUnsafeTTC)
        print('Maximum time to collision: ', maxUnsafeTTC)
        print('Average time to collision: ', avgUnsafeTTC)

        return minUnsafeTTC, maxUnsafeTTC, avgUnsafeTTC

    def compute_agent_TTC(self, vel_idx, heading_idx = None, pitch_idx = None):
        if self.workspaceDims > 1 and heading_idx == None:
            raise Exception('Must provide heading index for 2D or 3D scenario')
        if self.workspaceDims > 2 and pitch_idx == None:
            raise Exception('Must provide pitch index for 3D scenario')

        ego_trace = self.simulation_trace['agents'][self.egoID]['state_trace']
        
        self.agent_TTC = {}
        for agentID in list(self.simulation_trace['agents'].keys()):
            self.agent_TTC[agentID] = []
            agent_trace = self.simulation_trace['agents'][agentID]['state_trace']
            
            for i in range(len(ego_trace)):
                ego_state = ego_trace[i][1:]
                agent_state = agent_trace[i][1:]

                if self.workspaceDims == 1:
                    ego_vel_vec = np.array([ego_state[vel_idx], 0, 0])
                    agent_vel_vec = np.array([agent_state[vel_idx], 0, 0])
                    ego_pos_vec = np.array([ego_state[0], 0, 0])
                    agent_pos_vec = np.array([agent_state[0], 0, 0])
                elif self.workspaceDims == 2:
                    ego_vel_vec = np.array([cos(ego_state[heading_idx]), sin(ego_state[heading_idx]), 0])*ego_state[vel_idx]
                    agent_vel_vec = np.array([cos(agent_state[heading_idx]), sin(agent_state[heading_idx]), 0])*agent_state[vel_idx]
                    ego_pos_vec = np.array([ego_state[0], ego_state[1], 0])
                    agent_pos_vec = np.array([agent_state[0], agent_state[1], 0])
                elif self.workspaceDims == 3:
                    ego_vel_vec = np.array([cos(ego_state[heading_idx])*cos(ego_state[pitch_idx]), sin(ego_state[heading_idx])*cos(ego_state[pitch_idx]), sin(ego_state[pitch_idx])])*ego_state[vel_idx]
                    agent_vel_vec = np.array([cos(agent_state[heading_idx])*cos(agent_state[pitch_idx]), sin(agent_state[heading_idx])*cos(agent_state[pitch_idx]), sin(agent_state[pitch_idx])])*agent_state[vel_idx]
                    ego_pos_vec = np.array([ego_state[0], ego_state[1], ego_state[2]])
                    agent_pos_vec = np.array([agent_state[0], agent_state[1], agent_state[2]])

                pos_dif = agent_pos_vec - ego_pos_vec
                vel_dif = agent_vel_vec - ego_vel_vec

                t = pos_dif[0]/(-vel_dif[0])

                if pos_dif[1] == -vel_dif[1]*t and pos_dif[2] == -vel_dif[2]*t:
                    time_to_collision = t
                else:
                    time_to_collision = 1e20

                if t < 0:
                    time_to_collision = 1e20
                
                self.agent_TTC[agentID].append(time_to_collision)

        del self.agent_TTC[self.egoID]
        
        minUnsafeTTC = {}
        maxUnsafeTTC = {}
        avgUnsafeTTC = {}
        for unsafeID in list(self.agent_TTC.keys()):
            sum_TTC = 0
            len_TTC = 0
            for i in self.agent_TTC[unsafeID]:
                if i != 1e20:
                    sum_TTC += i
                    len_TTC += 1

            if len_TTC == 0:
                avgUnsafeTTC[unsafeID] = 1e20
            else:
                avgUnsafeTTC[unsafeID] = sum_TTC/len_TTC

            minUnsafeTTC[unsafeID] = min(self.agent_TTC[unsafeID])
            maxUnsafeTTC[unsafeID] = max(self.agent_TTC[unsafeID])            


        print('Agent time to collision metrics:')
        print('Minimum time to collision: ', minUnsafeTTC)
        print('Maximum time to collision: ', maxUnsafeTTC)
        print('Average time to collision: ', avgUnsafeTTC)

        return minUnsafeTTC, maxUnsafeTTC, avgUnsafeTTC

    def running_times(self):
        averageRT = sum(self.computation_times)/(len(self.computation_times))
        maxRT = max(self.computation_times)
        minRT = min(self.computation_times)
        
        return averageRT, maxRT, minRT

    def summary(self, showPlots = False):
        # Running time
        averageRT = sum(self.computation_times)/(len(self.computation_times))
        maxRT = max(self.computation_times)
        minRT = min(self.computation_times)

        print('Computation time metrics:')
        print('Average computation time: ', averageRT)
        print('Minimum computation time: ', minRT)
        print('Maximum computation time: ', maxRT)

        # Distance from unsafe sets
        minDist, maxDist, avgDists = self.compute_dists(showPlots)

        print('Distance from unsafe sets:')
        print('Average distance from unsafe sets: ', avgDists)
        print('Minimum distance from unsafe sets: ', minDist)
        print('Maximum distance from unsafe sets: ', maxDist)

        # Distance from agents
        agentMinDist, agentMaxDist, agentAvgDists = self.compute_agent_dists(showPlots)

        print('Distance from other agents:')
        print('Average distance from other agents: ', agentAvgDists)
        print('Minimum distance from other agents: ', agentMinDist)
        print('Maximum distance from other agents: ', agentMaxDist)

        # Untrusted controller usage
        untrusted_usage, safety_usage, num_switches = self.controllerUsage(showPlots)

        print('Controller usage metrics:')
        print('Untrusted controller usage: ', untrusted_usage)
        print('Safety controller usage: ', safety_usage)
        print('Number of switches: ', num_switches)
        
        if showPlots:
            plt.show()


# Helper functions:
def computeBallDist(ego_state,center,r): 
    dimension = len(center)
    if dimension == 1:
        return max(abs(ego_state[1]-center[0])-r,0)
    elif dimension == 2:
        return max(sqrt((ego_state[1]-center[0])**2+(ego_state[2]-center[1])**2)-r,0)
    elif dimension == 3:
        return max(sqrt((ego_state[1]-center[0])**2+(ego_state[2]-center[1])**2+(ego_state[3]-center[2])**2)-r,0)

def computeRectDist(ego_state,unsafe_rect): 
    center = unsafe_rect[0]
    dimension = len(center)
    dx = unsafe_rect[1]
    dy = 0 if dimension < 2 else unsafe_rect[2]
    dz = 0 if dimension < 3 else unsafe_rect[3]

    x = ego_state[1]
    x_min = center[0] - dx/2
    x_max = center[0] + dx/2

    if x_min <= x <= x_max:
        dist_x = 0
    else:
        dist_x = min(abs(x - x_min), abs(x - x_max))

    if dimension == 1:
        dist_y = 0
        dist_z = 0
    #dimension greater than 1
    else:
        y = ego_state[2]
        y_min = center[1] - dy/2
        y_max = center[1] + dy/2

        if y_min <= y <= y_max:
            dist_y = 0
        else:
            dist_y = min(abs(y - y_min), abs(y - y_max))
        
        if dimension == 2:
            dist_z = 0
        #dimension greater than 2
        else:
            z = ego_state[3]
            z_min = center[2] - dz/2
            z_max = center[2] + dz/2

            if z_min <= z <= z_max:
                dist_z = 0
            else:
                dist_z = min(abs(z - z_min), abs(z - z_max))
    
    return sqrt(dist_x**2 + dist_y**2 + dist_z**2)
