% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % A function that sets the Torque the Rover
% % Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


function [output, outputId] = setTorq(cmdId,tyreId,getTorque)

constVar;                   %   Calls the file constVar to get the number for each motor

cmdId = cmdId + 1;
if cmdId >= MAX_CMD_ID     
    cmdId = 1;              %%%%   WHATS HAPPENING HERE????
end





switch tyreId               %   Conditional statement saying 'if tyreID is the one inputted'
    case 1
        motorId = movLeft1;   %   Then is tyreID = 1 in inputs, put motorID in motor variable to be called later in sprintf
    case 2
        motorId = movRight1;  %   Similarily for all other cases  
    case 3
        motorId = movLeft2;
    case 4
        motorId = movRight2;
    case 5
        motorId = movLeft3;
    case 6
        motorId = movRight3;
    otherwise
        motorId = tyreId;
end        




%   Once the switch-case conditional is called, we set the command required
%   to the Rover. The 'sprintf' sets the number to a string

string = sprintf('>100,%d,DRIVE,%d,%d',cmdId, motorId, getTorque);

k = length(cmd) + 1;
tmp = cmd;
tmp{k} = string;    %%%%   WHAT HAPPENS HERE?????
output = tmp;       %%%%   IS THIS PUTTING STRINGS IN CELLS TO CALL LATER???
outputId = cmdId;

end







        
        
