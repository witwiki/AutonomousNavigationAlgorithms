% % % % % % % % % % % % % % % % % % % % % % % 
% A function that DRIVES the Rover
% Name: Vikram Udyawer
% SID: 30190672
% Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % 

function [] = driveRover(s,~)

speed = 20;
w = sprintf('>100,1,DRIVE,1,%d;>100,1,DRIVE,3,%d;>100,1,DRIVE,5,%d;>100,1,DRIVE,7,%d;>100,1,DRIVE,9,%d;>100,1,DRIVE,11,%d;',speed,speed,speed,speed,speed,speed);

fprintf(s,w); pause(0.05)
fprintf(s,w); pause(0.05)
fprintf(s,w); pause(0.05)
fprintf(s,w);
