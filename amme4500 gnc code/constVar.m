% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% A file that stores the constants needed for the simulation
% Name: Vikram Udyawer
% SID: 30190672
% Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%  Motor ID values for move

movLeft1 = 1;                 % These values are arbitrary motor numbers and need to be found out through experimentation on Monday
movLeft2 = 3;
movLeft3 = 5;

movRight1 = 2;
movRight2 = 4;
movRight3 = 6;


%%  Motor ID values for Steer

steeLeft1 = 7;
steeLeft2 = 9;
steeLeft3 = 11;

steeRight1 = 8;
steeRight2 = 10;
steeRight3 = 12;


%%  Stores the position of the wheels

%   x and y coordinate Positions of tyres at initial/rest position
posLeft1 = [-0.244095, 0.302205];           %8
posLeft2 = [-0.244095, -0.012505];          %10
posLeft3 = [-0.244095, -0.302205];          %12

posRight1 = [0.244095, 0.302205];           %2
posRight2 = [-0.244095,-0.012505];          %4
posRight3 = [0.244095, -0.302205];          %6

%%  Maximum Command ID for the Rover

%   This value is maximum and when calling the strings, it cannot be
%   greater than 3000 (not sure why - ask tutor?)
MAX_CMD_ID = 3000;      