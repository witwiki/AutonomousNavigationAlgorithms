% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % A function that sets the strings for Steer 
% % Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

function [output, outputId] = setSteer(cmd,cmdId,tyreId,getTorq)

constVar;

cmdId = cmdId + 1;
if cmdId >= MAX_CMD_ID
    cmdId = 1;                  
end


switch tyreId
    case 1
        motorId = steeLeft1;
    case 2
        motorId = steeRight1;
    case 3
        motorId = steeLeft2;
    case 4
        motorId = steeRight2;
    case 5
        motorId = steeLeft3;
    case 6
        motorId = steeRight3;
    otherwise
        motorId = tyreId;
end

str = sprintf('>100,%d,STEER,%d,%d',cmdID,motorId,getTorq);

k = length(cmd) + 1;
tmp = cmd;                  %%%%% WHATS HAPPENING HERE?????
tmp{k} = str;
output = tmp;
outputId = cmdId;

end

     