clear all; close all; clc;

s = serial('COM3');
set(s,'BaudRate',57600,'DataBits',8,'Parity','none','StopBits',1,'FlowControl','none');

fopen(s);
fprintf(s,'>100,121,MICROCONTROLLER,ENABLE;');
%%
fprintf(s,'>100,122,LASER,OFF;');
%%
fprintf(s,'>100,122,LASER,ON;');
%%
fprintf(s,'>100,123,TILT,00;');
%%
fprintf(s,'>100,123,PAN,10;');
%%
speed = 20;
command = sprintf('>100,1,DRIVE,1,%d;>100,1,DRIVE,3,%d;>100,1,DRIVE,5,%d;>100,1,DRIVE,7,%d;>100,1,DRIVE,9,%d;>100,1,DRIVE,11,%d;',speed,speed,speed,speed,speed,speed);
fprintf(s,command); pause(0.05)
fprintf(s,command); pause(0.05)
fprintf(s,command);
%%
angle = 90;
command = sprintf('>100,123,STEER,2,%d;>100,123,STEER,4,%d;>100,123,STEER,6,%d;>100,123,STEER,8,%d;>100,123,STEER,10,%d;>100,123,STEER,12,%d;',angle,angle,angle,angle,angle,angle);
fprintf(s,command); pause(0.05)
fprintf(s,command); pause(0.05)
fprintf(s,command);
%% 
fprintf(s,'>100,123,RESETMOTORS;');
%% Stop command
fprintf(s,'>100,123,STOP;');
%%
fclose(s);

delete(s)

clear s