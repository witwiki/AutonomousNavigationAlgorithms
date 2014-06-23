#######################################################################
#
#   @witwiki #2231.12062014
#   Basic 3D PD Controller for the AR Parrot
#   using the 'quadrotor.command' package
#
#   (1) Plots the current state and desired set points
#   (2) Initialises the PD controller as a matrix of 3 by 1
#   (3) Calculates the error in the response for both proportional
#       and derivative.
#   (4) Next calculates the proportional and derivative controls P
#       and D
#   (5) Calculates the full control, U
#   (6) Finally, updates the orginal/previous error initialized in the 
#       class UserCode.
#
#######################################################################

import numpy as np

class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))

class UserCode:
    def __init__(self):
        #: tune gains
    
        # xy control gains
        Kp_xy = 0.5 # xy proportional
        Kd_xy = 0.0 # xy differential
        
        # height control gains
        Kp_z  = 0.5 # z proportional
        Kd_z  = 0.0 # z differential
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T
    
    def compute_control_command(self, t, dt, state, state_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param state: State - current quadrotor position and velocity computed from noisy measurements
        :param state_desired: State - desired quadrotor position and velocity
        :return - xyz velocity control signal represented as 3x1 numpy array
        '''
        # plot current state and desired setpoint
        self.plot(state.position, state_desired.position)
        
        #: implement PID controller computing u from state and state_desired
        u = np.zeros((3,1))
        
        # Calculating the error in response for proportional and derivative controls
        presErr = state_desired.position - state.position
        kdErr = state_desired.velocity - state.velocity
        #kiErr = (state_desired.position - state.position) * dt
        
        # Calculating the Proportional and Derivative Controls
        p = self.Kp * presErr
        #i = self.Ki * kiErr
        d = self.Kd * kdErr
        
        # Calculating the full PD controller
        u = p + d #+ i
        
        # Updating the error for each time step
        self.prevErr = presErr      
        
        return u
        
    def plot(self, position, position_desired):
        from plot import plot
        plot("x", position[0])
        plot("x_des", position_desired[0])
        plot("y", position[1])
        plot("y_des", position_desired[1])
        plot("z", position[2])
        plot("z_des", position_desired[2])
        
