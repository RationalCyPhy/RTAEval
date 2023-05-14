import sys
from datetime import datetime
from typing import List

import matplotlib.pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3

class simpleSim:
    def __init__(self):
        # Set vis type
        self.vis = True
        self.plotType = "2D"
        self.simType = "2D"
        
        # Setting axes limits
        self.ref_agent_id = None
        self.axes_type = None

        self.axes_xlim = None
        self.axes_ylim = None
        self.axes_zlim = None

        # Set time params
        self.dt = 0.1
        self.T = self.dt*100

        # Agents in the scenario
        self.agents = []
        self.RTAs = []
        self.modes = []
        self.init_states = []

        self.unsafe_sets = []

    def addAgents(self, agents:List, modes:List, RTAs:List, initStates:List) -> None:
        # Add agents to the scenario
        if len(agents) != len(RTAs) or len(agents) != len(modes) or len(agents) != len(initStates) or len(RTAs) != len(modes) or len(RTAs) != len(initStates) or len(modes) != len(initStates):
            raise Exception("Number of RTAs, modes, initial states, and agents must be equal!")
        else:
            for agent_num in range(len(agents)):
                agent = agents[agent_num]
                RTA = RTAs[agent_num]
                mode = modes[agent_num]
                initState = initStates[agent_num]

                self.agents.append(agent)
                self.RTAs.append(RTA)
                self.modes.append(mode)
                self.init_states.append(initState)

    def addUnsafeSets(self, unsafe_sets:List) -> None:
        self.unsafe_sets = unsafe_sets

    def setSimType(self, vis:bool = True, plotType:str = "2D", simType:str = "2D") -> None:
        # Set sim type for the scenario
        self.vis = vis
        if plotType not in ["2D", "3D"]:
            raise Exception("plotType must be 2D or 3D!")
        else:
            self.plotType = plotType
        if simType not in ["1D", "2D", "3D"]:
            raise Exception("simType must be 1D, 2D, or 3D!")
        else:
            self.simType = simType
    
    def chooseAxesLimits(self, axes_type:str, ref_agent_id:str, axes_xlim:list, axes_ylim:list, axes_zlim:list) -> None:
        if axes_type not in ["relative","fixed"]:
            raise Exception("axes_type must be relative or fixed!")
        
        self.ref_agent_id = ref_agent_id
        self.axes_type = axes_type

        self.axes_xlim = axes_xlim
        self.axes_ylim = axes_ylim
        self.axes_zlim = axes_zlim

    def setAxesLimits(self):
        if self.axes_type == "fixed":
            self.ax.set_xlim(self.axes_xlim)
            self.ax.set_ylim(self.axes_ylim)
            if self.plotType == "3D":
                self.ax.set_zlim(self.axes_zlim)
        elif self.axes_type == "relative":
            try:
                ref_state = self.simTrace["other"][self.ref_agent_id]["state_trace"][-1][1:]
            except:
                ref_state = self.simTrace["agents"][self.ref_agent_id]["state_trace"][-1][1:]
            
            self.ax.set_xlim([ref_state[0] + self.axes_xlim[0], ref_state[0] + self.axes_xlim[1]])
            if self.simType == "1D":
                self.ax.set_ylim(self.axes_ylim)
            elif self.simType == "2D" or self.simType == "3D":
                self.ax.set_ylim([ref_state[1] + self.axes_ylim[0], ref_state[1] + self.axes_ylim[1]])
                if self.plotType == "3D":
                    self.ax.set_zlim(self.axes_zlim) 


    def setTimeParams(self, dt:float = 0.1, T:float = 100) -> None:
        self.dt = dt
        self.T = T

    def initializeSimTrace(self) -> None:
        self.simTrace = {"agents":{}, "unsafe":{}} 

        for agent_num in range(len(self.agents)):
            agent = self.agents[agent_num]
            agentMode = self.modes[agent_num]
            agentInitState = self.init_states[agent_num]

            self.simTrace["agents"][agent.id] = {"state_trace":[[0]+agentInitState], "mode_trace":[agentMode]}

        for unsafe_set in self.unsafe_sets:
            self.simTrace["unsafe"][unsafe_set.id] = {"set_type":unsafe_set.set_type, "state_trace":[[0, unsafe_set.update_def(self.simTrace)]]}

    def updateSimState(self, timeStep) -> None:
        # Update agent states

        currStates = []
        for agent in self.agents:
            agentState = self.simTrace["agents"][agent.id]["state_trace"][-1][1:]
            currStates.append(agentState)
        
        for agent_num in range(len(self.agents)):
            agent = self.agents[agent_num]
            agentRTA = self.RTAs[agent_num]
            agentMode = self.simTrace["agents"][agent.id]["mode_trace"][-1] 

            agentState = self.simTrace["agents"][agent.id]["state_trace"][-1][1:]

            if agentRTA != None:
                new_mode = agentRTA.RTASwitch(self.simTrace)
            else:
                new_mode = agentMode
            
            # Save state to simulation trace
            self.simTrace["agents"][agent.id]["mode_trace"].append(new_mode)

        for agent_num in range(len(self.agents)):
            agent = self.agents[agent_num]
            agentMode = self.simTrace["agents"][agent.id]["mode_trace"][-1] 
            agentState = self.simTrace["agents"][agent.id]["state_trace"][-1][1:]           

            # Update ego state
            new_state = agent.step([agentMode], agentState, self.dt, simulatorState=self.simTrace)

            # Save state to simulation trace
            agentStateToAppend = [timeStep*self.dt]
            agentStateToAppend.extend(new_state)
            self.simTrace["agents"][agent.id]["state_trace"].append(agentStateToAppend)
             
        for unsafe_set in self.unsafe_sets:
            new_set_def = [timeStep*self.dt, unsafe_set.update_def(self.simTrace)]
            self.simTrace["unsafe"][unsafe_set.id]["state_trace"].append(new_set_def)

        if self.vis:
            if timeStep >= int(self.T/self.dt) -1:
                # with open('multiFollower.pkl', 'wb') as fp:
                #     pickle.dump(self.simTrace, fp)

                # print(self.simTrace)
                sys.exit()
            
            self.ax.clear()

            for unsafe_set in self.unsafe_sets:
                unsafe_set.plot_set(self.ax, self.plotType)

            for agent_num in range(len(self.agents)):
                agent = self.agents[agent_num]
                agentRTA = self.RTAs[agent_num]
                agentMode = self.modes[agent_num]

                agentState = self.simTrace["agents"][agent.id]["state_trace"][-1][1:]
                agentMode = self.simTrace["agents"][agent.id]["mode_trace"][-1]
                
                if agentMode.value == 3:
                    agentColor = "blue"
                elif agentMode.value == 2:
                    agentColor = "orange"
                else:
                    agentColor = "black"

                x = agentState[0]
                ego_goal_x = agent.goal_state[0]
                if self.simType == "1D":
                    y = 0
                    ego_goal_y = 0
                    z = 0
                    ego_goal_z = 0
                else:
                    y = agentState[1]
                    ego_goal_y = agent.goal_state[1]
                    if self.simType == "2D":
                        z = 0
                        ego_goal_z = 0
                    else:
                        z = agentState[2]
                        ego_goal_z = agent.goal_state[2]

                if self.plotType == "2D":
                    self.ax.plot(x, y, marker="o", color = agentColor)
                    if 'ego' in agent.id:
                        self.ax.plot(ego_goal_x, ego_goal_y, marker="*", color = 'green')
                else:
                    self.ax.plot(x, y, z, marker="o", color = agentColor)
                    if 'ego' in agent.id:
                        self.ax.plot(ego_goal_x, ego_goal_y, ego_goal_z, marker="*", color = 'green')

            
            
            self.setAxesLimits()


    def runSim(self, saveVid:bool = False, vidFileName:str = r"sim_"+str(datetime.now())+"_.mp4") -> dict:
        # Initialize simulation trace:
        self.initializeSimTrace()

        if self.vis:
            if self.plotType == "2D":
                self.fig, self.ax = plt.subplots()
            else:
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection='3d')

            ani = animation.FuncAnimation(self.fig, self.updateSimState, interval = 10, frames = int(self.T/self.dt))
            
            if saveVid:
                f = vidFileName
                writervideo = animation.FFMpegWriter(fps=10) 
                ani.save(f, writer=writervideo)

            plt.show()

        else:
            timeStep = 0
            while timeStep*self.dt <= self.T:
                self.updateSimState(timeStep)
                timeStep += 1
        return self.simTrace