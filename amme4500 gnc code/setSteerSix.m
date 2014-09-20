% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% A function that iterates the steer six times so to make sure each wheel
% gets steered.
% Name: Vikram Udyawer
% SID: 30190672
% Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


function [outputCmd, outputCmdId] = setSteerSix(cmd, cmdId, getTorq)

tmp1 = cmd;             
tmp2 = cmdId;

for i = 1:6
    [tmp1 tmp2] = setSteer(tmp1,tmp2,i,getTorq);      %   All this does is calls 'setSteer' six times to activate all wheels to Steer
end

outputCmd = tmp1;       
outputCmdId = tmp2;
end

