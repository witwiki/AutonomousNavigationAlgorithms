% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % A function that sets the strings for a system reset
% % Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


function [output,outputId] = setReset(cmd,cmdId)
% This helps generate a reset command

constVar;

cmdId = cmdId + 1;
if cmdId >= MAX_CMD_ID
    cmdId = 1;                  % This is check to make sure the cmdId does not exceed 3000. This is an arbitrary limit for the Rover
end

str = sprintf('>100,%d,RESETMOTORS;',cmdId);

j = length(cmd) + 1;
tmp = cmd;
tmp{j} = str;
output =tmp;
outputId = cmdId;

end
