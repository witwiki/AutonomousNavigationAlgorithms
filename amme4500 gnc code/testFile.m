% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % A test file that tests our functions
% % Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


clear all; clc

cmd{1} = 0;
cmdId = 0;

[cmd cmdId] = setTorq(cmd,cmdId,1,10);

[cmd cmdId] = setSteerSix(cmd,cmdId,30);

[cmd cmdId] = setTorqSix(cmd,cmdId,100);

[cmd cmdId] = stop(cmd,cmdId,100);

[cmd cmdId] = setReset(cmd,cmdId);

posA = [1 1]';
[cmd cmdId] = setSteerArcTurn(cmd,cmdId, posA);
ratio = calSpeedRatio(posA);

sendCmd(0,cmd);

