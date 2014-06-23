#######################################################################
#
#   @witwiki #2032.03062014
#   Basic 1D PD Controller for the AR Parrot
#   using the 'quadrotor.command' package
#
#   (1) Calculates the error in the response for both proportional
#       and derivative.
#   (2) Next calculates the proportional and derivative controls P
#       and D
#   (3) Calculates the full control, U
#   (4) Finally, updates the orginal/previous error initialized in the 
#       class UserCode.
#
#######################################################################

class UserCode:
    def __init__(self):
        #: Tune gains
        self.Kp = 100
        self.Kd = 20
        self.prevErr = 0.0
            
    def compute_control_command(self, t, dt, x_measured, x_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to compute_control_command
        :param x_measured: measured position (scalar)
        :param x_desired: desired position (scalar)
        :return - control command u
        '''
        #: implement PD controller
        u = 0.0
        
        # Calculating the error in response for proportional and derivative controls
        presErr = x_desired - x_measured            
        kdErr = (presErr - self.prevErr)/dt
        
        # Calculating the Proportional and Derivative Controls
        p = self.Kp * presErr
        d = self.Kd * kdErr
        
        # Calculating the full PD controller
        u = p + d
        
        # Updating the error for each time step
        self.prevErr = presErr        
        
        return u

