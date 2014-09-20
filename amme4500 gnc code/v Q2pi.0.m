% ------------------------------------------------------------------ %
%
%   Vikram Udyawer
%   306190672
%   Guidance & Control
%   Assignment 2 2012
%
% ------------------------------------------------------------------ %

%Main file to generate waypoints and straight line path

% Straight Line path generation
clear all;
clc;
close all;

%Construct waypoints
wp = [0,0;3,1;7,3;10,2;17,3;10,4;1,2;21,2;25,5];

%Define start and end points
pos.startx = wp(1,1);
pos.starty = wp(1,2);
pos.starttheta = 60*pi/180;

pos.endx = 25;
pos.endy = 5;
pos.endtheta = 0;

%Generate path between waypoints
for i = 1:length(wp)-1
    m = (wp(i+1,2)-wp(i,2))/(wp(i+1,1)-wp(i,1));
    x{i} = wp(i,1):(wp(i+1,1)-wp(i,1))/100:wp(i+1,1);
    yy{i} = m.*x{i} + wp(i,2) - m.*wp(i,1);
end

%Plot Waypoints, start, end position and staright line path
figure;
for i = 1:length(wp)
    plot(wp(i,1),wp(i,2),'ro');
    hold on
    plot(pos.startx,pos.starty,'g*')
    plot(pos.endx,pos.endy,'g*');
    grid on;
    set(gca,'xtick',[0:1:25]);
    set(gca,'ytick',[0:1:5]);
end
hold on;
for i = 1:length(wp)-1
    plot(x{i}(:),yy{i}(:),'-');
end

%% Generate Kinematic model of Ground Vehicle

V = 0.5;                    %Vehicle constant velocity (m/s)
phi_max = 45*pi/180;        %Maximum steering angle (rad)
phi_dot_max = 40*pi/180;    %Maximum turn rate (rad/s)
Store.x = [];
Store.y = [];
Store.theta = [];
Store.distanceerror = [];
Store.headingerror = [];
Store.t = [];

t_step = 0.1;               %Time step
t_final = 400;              %Simulation time

%Initialise vehicle position
pos.x = pos.startx;
pos.y = pos.starty;
pos.theta = pos.starttheta;
t = 0;

%Initialise controller gains
% % Option 1
% kp_d = -5;
% kd_d = 1;
% ki_d = -2;
% kp_heading = -4;
% kd_heading = 1.5;
% ki_heading = -3;

% % Option 2
% kp_d = -3;
% kd_d = 0.5;
% ki_d = -1;
% kp_heading = -2;
% kd_heading = 0.5;
% ki_heading = -1.5;

% Option 3
kp_d = -2.5;
kd_d = 0.3;
ki_d = -0.5;
kp_heading = -1.5;
kd_heading = 0.05;
ki_heading = -0.5;

%Initialise error values for derivative calculation
pos.distanceerror = 0;
pos.headingerror = 0;
pos.distanceintegral = 0;
pos.headingintegral = 0;

%Initialise waypoint counter
wp_current = 1;
flag = 0;

for i = 1:length(wp)-1
    
    while t < t_final

        %Store initialised error values
        Store.distanceerror = [Store.distanceerror pos.distanceerror];
        Store.headingerror = [Store.headingerror pos.headingerror];
        Store.x = [Store.x pos.x];
        Store.y = [Store.y pos.y];
        error_d = pos.distanceerror;
        error_heading = pos.headingerror;
        Store.t = [Store.t t];
        Store.theta = [Store.theta pos.theta];

        %Update vehicle position based on velocity
        pos.x = V*t_step*cos(pos.theta) + pos.x;
        pos.y = V*t_step*sin(pos.theta) + pos.y;

        %Determine heading error of vehicle
        pos.targetheading = atan2((wp(i + 1,2) - wp(i,2)),...
            wp(i + 1,1) - wp(i,1));
        
        if pos.targetheading > 2*pi
            pos.targetheading = pos.targetheading - 2*pi;
%         elseif pos.targetheading < 0
%             pos.targetheading = pos.targetheading + 2*pi;
        end
        
        pos.headingerror = pos.theta-pos.targetheading;
        
        if pos.headingerror > 2*pi
            pos.headingerror = pos.headingerror - 2*pi;
%         elseif pos.headingerror < 0
%             pos.headingerror = pos.headingerror + 2*pi;
        end

        %Determine perpendicular distance error
        pos.distanceerror = -sin(pos.targetheading)*(pos.x - wp(i,1)) + ...
            cos(pos.targetheading)*(pos.y - wp(i,2));

%         %Determine error derivatives
%         pos.distancederivative = (pos.distanceerror - error_d)/t_step;
%         pos.headingderivative = (pos.headingerror - error_heading)/t_step;
%         
        %Determine error intgerals
        pos.distanceintegral = pos.distanceerror*t_step + pos.distanceintegral;
        pos.headingintegral = pos.headingerror*t_step + pos.headingintegral;
        
        %Update steering set point - Multiply derivative terms by flag, 0
        %if on step where a waypoint is changing otherwise you get
        %discontinuity and it will likely follow the wrong path
%         pos.phi_sp = kp_d*pos.distanceerror + kd_d*pos.distancederivative*flag + ...
%             kp_heading*pos.headingerror + kd_heading*pos.headingderivative*flag;

        pos.phi_sp = kp_d*pos.distanceerror + ki_d*pos.distanceintegral*flag + ...
            kp_heading*pos.headingerror + ki_heading*pos.headingintegral*flag;

        %Impose steering limits
        if pos.phi_sp > phi_max
            pos.phi_sp = phi_max;
        elseif pos.phi_sp < -phi_max
            pos.phi_sp = -phi_max;
        end
        
        %Update car heading angle by commanded steering
        pos.theta = pos.theta + pos.phi_sp;
        
        if pos.theta > 2*pi
            pos.theta = pos.theta - 2*pi;
%         elseif pos.theta < 0
%             pos.theta = pos.theta +2*pi;
        end
        
        %Check if current next waypoint has been attained
        waypoint_dist = sqrt((wp(i + 1,1) - pos.x)^2 + ...
            (wp(i + 1,2) - pos.y)^2);
        
        flag = 1;
        if waypoint_dist < 0.1
            flag = 0;
            break
        end

        %Increment time step
        t = t + t_step;
       
    end

end

%Plot path planned by rover
figure
plot(Store.x,Store.y,'r--','LineWidth',1.5);
grid on
set(gca,'xtick',[0:1:25]);
set(gca,'ytick',[0:1:5]);
hold on

%Initialise handle for rover simulation
h2 = plot(0,0,'ro');
axis([0 25 0 5]);

for i = 1:length(wp)
    plot(wp(i,1),wp(i,2),'bo');
    hold on
    plot(pos.startx,pos.starty,'g*')
    plot(pos.endx,pos.endy,'g*');
    grid on;
    set(gca,'xtick',[0:1:25]);
    set(gca,'ytick',[0:1:5]);
end
for i = 1:length(wp)-1
    plot(x{i}(:),yy{i}(:),'-');
end

%Simulate rover motion
for j = 1:length(Store.x)
    set(h2,'xdata',Store.x(j),'ydata',Store.y(j),'LineWidth',5);
    pause(0.01);
end
hold off

figure
%Plot heading and Path Errors
plot(Store.t,Store.distanceerror);
title('Rover Path Errors (Signed)');
xlabel('Time (s)');
ylabel('Path Error (m)');
grid on

figure
plot(Store.t,abs(Store.distanceerror));
title('Rover Path Errors (Absolute)');
xlabel('Time (s)');
ylabel('Path Error (m)');
grid on

figure
plot(Store.t,Store.headingerror*180/pi);
title('Rover Heading Errors (Signed)');
xlabel('Time (s)');
ylabel('Heading Error (degrees)');
grid on

figure
plot(Store.t,abs(Store.headingerror*180/pi));
title('Rover Heading Errors (Absolute)');
xlabel('Time (s)');
ylabel('Heading Error (degrees)');
grid on



