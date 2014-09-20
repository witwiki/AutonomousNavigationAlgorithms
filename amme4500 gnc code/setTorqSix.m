% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% A function that iterates the torque six times so to make sure each wheel
% gets a torque.
% Name: Vikram Udyawer
% SID: 30190672
% Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


function [outputCmd, outputCmdId] = setTorqSix(cmd, cmdId, setTorq)

tmp1 = cmd;         %%%%%   WHATS HAPPENING HERE???
tmp2 = cmdId;

for i = 1:6
    [tmp1 tmp2] = setTorq(tmp1,tmp2,i,getTorq);      %   All this does is calls 'setTorque' six times to activate all wheels to move forward
end


outputCmd = tmp1;       %%%%%   WHATS HAPPENING HERE???
outputCmdId = tmp2;

end

