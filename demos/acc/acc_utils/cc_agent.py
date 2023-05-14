# Dubin's aircraft agent.
from typing import List
from math import cos, sin, atan2, sqrt, pi

import numpy as np
from scipy.integrate import odeint
from verse import BaseAgent
from verse import LaneMap

import sys

class ccAgent(BaseAgent):
    def __init__(self, id, ctrlArgs = ('no control', False, None), code=None, file_name=None, initial_state=None, initial_mode=None, static_param=None, uncertain_param=None):
        '''Inputs for the deputy agent are as follows:
            id (str): id of the agent
            ctrlArgs (tuple): arguments to run simulation of the agent

            EXACTLY ONE OF THE FOLLOWING SHOULD BE GIVEN:
                file_name (str): file_name that points to the mode control
                code (str): Python string defining the control ??
        '''

        # Calling the constructor of tha base class
        super().__init__(id, code, file_name, initial_state, initial_mode, static_param, uncertain_param)

        # Set the control
        if ctrlArgs[0] == 'no control':
            self.control = self.no_control
        elif ctrlArgs[0] == 'proportional':
            self.control = self.P
        elif ctrlArgs[0] == 'pid':
            self.control = self.PID
        elif ctrlArgs[0] == 'bangbang1d':
            self.control = self.BangBang1D
        elif ctrlArgs[0] == 'bangbang1d weighted':
            self.control = self.BangBang1D_weighted
        elif ctrlArgs[0] == 'bangbang2d':
            self.control = self.BangBang2D
        else:
            self.control = self.BangBang2D_weighted

        self.follower = ctrlArgs[1]
        self.desired_traj = ctrlArgs[2] #Function that defines the desired trajectory of the follower agent

        self.Kp = [1,0.5]
    
        self.Ki = [0.2,0.5]
        self.Kd = [0.2,0.5]

        self.a_max = 3
        self.v_max = 5
        self.x_err_max = 3
        self.v_err_max = 5
        self.k = 0.6

        self.state_hist = []
        self.input_hist = []
        self.error_hist = []

        self.goal_state = [0,0]

        # Needed for TCSimulate
        self.predictedSimulation = None

    # Step function
    def next_state(self, curr_state, curr_input, time_step):
        x_curr, v_curr = curr_state
        a_curr = curr_input

        if abs(a_curr) > self.a_max:
            a_curr = np.sign(a_curr)*self.a_max


        x_next = x_curr + v_curr*time_step
        v_next = v_curr + a_curr*time_step

        if abs(v_next) >= self.v_max:
            v_next = np.sign(v_next)*self.v_max

        return [x_next, v_next]

    # controls
    def no_control(self, time_step):
        return 0

    def P(self, time_step):
        k1 = self.Kp[0]
        k2 = self.Kp[1]
        xrel = self.goal_state[0] - self.state_hist[-1][0]
        vrel = self.goal_state[1] - self.state_hist[-1][1]
        return k1*xrel + k2*vrel


    def PID(self, time_step):
        x_err_curr = self.goal_state[0] - self.state_hist[-1][0]
        v_err_curr = self.goal_state[1] - self.state_hist[-1][1]

        try: 
            x_err_curr, v_err_curr = self.error_hist[-1]
        except:
            x_err_curr, v_err_curr = (0, 0)
        try: 
            x_err_prev, v_err_prev = self.error_hist[-2]
        except:
            x_err_prev, v_err_prev = (0, 0)
        try:
            x_err_2prev, v_err_2prev = self.error_hist[-3]
        except:
            x_err_2prev, v_err_2prev = (0, 0)

        try:
            accel_prev = self.input_hist[-1]
        except:
            accel_prev = 0

        Kp = self.Kp
        Ki = self.Ki
        Kd = self.Kd

        dt = time_step

        term1 = accel_prev 
        term2 = (Kp[0] + Ki[0]*dt + (Kd[0]/dt))*x_err_curr + (Kp[1] + Ki[1]*dt + (Kd[1]/dt))*v_err_curr
        term3 = (-Kp[0] - (2*Kd[0]/dt))*x_err_prev + (-Kp[1] - (2*Kd[1]/dt))*v_err_prev
        term4 = (Kd[0]/dt)*x_err_2prev + (Kd[1]/dt)*v_err_2prev

        a_curr = term1 + term2 + term3 + term4

        return a_curr

    def BangBang1D(self, time_step):
        x_err_curr = self.goal_state[0] - self.state_hist[-1][0]
        v_err_curr = self.goal_state[1] - self.state_hist[-1][1]

        a_curr = np.sign(x_err_curr)*self.a_max
        if abs(a_curr) > self.a_max:
            a_curr = np.sign(a_curr)*self.a_max

        return a_curr

    def BangBang2D(self, time_step):
        x_err_curr = self.goal_state[0] - self.state_hist[-1][0]
        v_err_curr = self.goal_state[1] - self.state_hist[-1][1]

        a_curr = (self.k*np.sign(x_err_curr) + (1-self.k)*np.sign(v_err_curr))*self.a_max
        if abs(a_curr) > self.a_max:
            a_curr = np.sign(a_curr)*self.a_max

        return a_curr

    def BangBang1D_weighted(self, time_step):
        x_err_curr = self.goal_state[0] - self.state_hist[-1][0]
        v_err_curr = self.goal_state[1] - self.state_hist[-1][1]

        a_curr = (x_err_curr/self.x_err_max)*self.a_max
        if abs(a_curr) > self.a_max:
            a_curr = np.sign(a_curr)*self.a_max

        return a_curr

    def BangBang2D_weighted(self, time_step):
        x_err_curr = self.goal_state[0] - self.state_hist[-1][0]
        v_err_curr = self.goal_state[1] - self.state_hist[-1][1]

        a_curr = (self.k*(x_err_curr/self.x_err_max) + (1-self.k)*(v_err_curr/self.v_err_max))*self.a_max
        if abs(a_curr) > self.a_max:
            a_curr = np.sign(a_curr)*self.a_max

        return a_curr

    def simulate(self, mode: List[str], initialCondition, time_horizon, time_step):
        time = time_step
        init_state = initialCondition

        if not bool(self.state_hist):
            self.state_hist.append(init_state)
            if self.follower:
                self.error_hist.append([self.goal_state[0]-init_state[0], self.goal_state[1]-init_state[1]])
                
        if self.follower:
            if mode[0].value == 1:
                self.control = self.no_control
            elif mode[0].value == 2:
                self.control = self.BangBang1D
            else:
                self.control = self.P

        while time < time_horizon:
            curr_input = self.control(time_step)
            init_state = self.next_state(init_state, curr_input, time_step)

            self.state_hist.append(init_state)
            self.input_hist.append(curr_input)

            if self.follower:
                self.error_hist.append([self.goal_state[0]-init_state[0], self.goal_state[1]-init_state[1]])
                self.goal_state = self.next_state(self.goal_state, 0, time_step)

            time += time_step

        
    def step(self, mode: List[str], initialCondition, time_step, simulatorState = None):
        if self.follower:
            self.goal_state = self.desired_traj(simulatorState)
            if mode[0].value == 1:
                self.control = self.no_control
            elif mode[0].value == 2:
                self.control = self.BangBang1D
            else:
                self.control = self.P
                
            self.simulate(mode, initialCondition, 2*time_step, time_step)

        else:
            self.simulate(mode, initialCondition, 2*time_step, time_step)
            
        return self.state_hist[-1]

    def TC_simulate(self, mode: List[str], initialCondition, time_horizon, time_step, lane_map: LaneMap = None) -> np.ndarray:            
        trace = []
        time = 0
        init_state = initialCondition

        if self.follower:
            self.goal_state = self.desired_traj(self.predictedSimulation)
        
        if not bool(self.state_hist):
            self.state_hist.append(init_state)
            if self.follower:
                self.error_hist.append([self.goal_state[0]-init_state[0], self.goal_state[1]-init_state[1]])

        while time < time_horizon:
            trace.append([time] + init_state)
            curr_input = self.control(time_step)
            init_state = self.next_state(init_state, curr_input, time_step)

            self.state_hist.append(init_state)
            self.input_hist.append(curr_input)

            if self.follower:
                self.error_hist.append([self.goal_state[0]-init_state[0], self.goal_state[1]-init_state[1]])
                self.goal_state = self.next_state(self.goal_state, 0, time_step)

            time += time_step

        return np.array(trace)
