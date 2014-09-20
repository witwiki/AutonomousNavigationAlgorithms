% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% A function that sets the strings for Stop
% Name: Vikram Udyawer
% SID: 30190672
% Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [output,outputId] = setStop(cmd,cmdId)
% This generates the Stop command
constVar;

cmdId = cmdId + 1;
if cmdId >= MAX_CMD_ID
    cmdId = 1;
end

setCmd = sprintf('>100,%d,STOP;',cmdId);

k = length(cmd) + 1;
tmp = cmd;
tmp{k} = setCmd;
output = tmp;
outputId = cmdId;

end

    
        