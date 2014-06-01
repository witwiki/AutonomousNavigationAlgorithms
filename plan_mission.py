#############################################
#
#	@witwiki #2032.23052014
#	Basic motion commands for the AR Parrot
#	using the 'quadrotor.command' package
#
#############################################
import quadrotor.command as cmd
from math import sqrt

def plan_mission(mission):

    commands  = [
        cmd.down(0.5),
        cmd.right(1),
        cmd.turn_left(45),
        cmd.forward(sqrt(2)),
        cmd.turn_right(45),
        cmd.right(1),
        cmd.turn_left(45),
        cmd.forward(sqrt(0.5)),
        cmd.turn_left(90),
        cmd.forward(sqrt(0.5)),
        cmd.turn_left(45),
        cmd.forward(1),
        cmd.turn_right(45),
        cmd.backward(sqrt(2)),
        cmd.turn_left(45),
        cmd.forward(1),
    ]
    
    #mission.add_commands(commands)
    
    myCommand = [
        cmd.up(1),
        cmd.right(2),
        cmd.forward(5),
        cmd.left(2),
        cmd.backward(5),
        cmd.left(2),
        cmd.forward(5),
        cmd.right(2),
        cmd.turn_right(360),
        cmd.backward(5),
        cmd.turn_left(360),
        cmd.down(1),
    ]
    
    mission.add_commands(myCommand)