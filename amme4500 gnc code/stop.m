% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % A function that decreases torque from current torque aka input torque to
% % zero
% Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [outputCmd,outputCmdId] = stop(cmd,cmdId,inputTorq)

% This decreases the Torque from current torque (inputTorq) to zero

tmp1 = cmd;
tmp2 = cmdId;

% The first for loop sets the decrease rate of the torque to zero while the
% second for loop calls the setTorq function to set the torque to the value
% of the calculated torque at each decrease set.
decrease = 15;
for torque = (inputTorq - decrease):-decrease:0
    for i = 1:6
        [tmp1 tmp2] = setTorq(tmp1,tmp2,i,torque);
    end
end

% This eventually sets the stop command for the command and commandId
[tmp1 tmp2] = setStop(tmp1,tmp2);
outputCmd = tmp1;
outputCmdId = tmp2;

end

