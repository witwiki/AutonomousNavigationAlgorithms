#################################################################
#
#   @witwiki #1423.30052014
#   Plots the quadrotor's path using the 
#   plot_trajectory function. 
#
#   (1) Converts velocity data from navdata variable to position
#       parameters.
#   (2) Then converts local to global coordinates using the 
#       transformation matrix to account for both rotation and 
#       translations.
#   (3) Finally, self.position is updated which is what's plotted
#
#################################################################
import numpy as np
from plot import plot_trajectory
from math import cos, sin

class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        
        # Translation coordinates x and y
        posX = np.multiply(navdata.vx, dt)
        posY = np.multiply(navdata.vy, dt)
        
        # Rotation about Z axis
        rotPsi = navdata.rotZ
        
        # Matrix of Quadcopter's local position
        localPos = np.array([[posX],
                             [posY]])
        
        # Transformation matrix to transform local to global coordinates
        rotMatrix = np.array([[cos(rotPsi), -sin(rotPsi)], 
                           [sin(rotPsi), cos(rotPsi)]])
        
        # Transform from local to global
        globalPos = np.dot(rotMatrix, localPos)
        
        # Quadcopter Global coordinates position update
        self.position += globalPos
        
        
        # TODO: update self.position by integrating measurements contained in navdata
        plot_trajectory("odometry", self.position)
