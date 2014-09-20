% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % A function that sets the strings for a system reset
% % Name: Vikram Udyawer
% % SID: 30190672
% % % Project 2, S2 2012
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


function [angle] = calcAngle(posTyre,posA)

% Vector from tyre to point A around the Rover vacinity.
% This helps find the instantaneous centre
vec = posA - posTyre;
vecp = [vec(2) - vec(1)]';  %   Normal vector of A
if dot(vecp, [0 1]') < 0
    vecp = -vecp;           %   Make sure it points forward
end

angle = atan2(vecp(2),vecp(1));
angle = princAng(angle);    %   Converts to Principal Values
angle = angle/pi*180;       %   Converts to Degrees
angle = 90 - angle;         %   The angle needs to steer
angle = round(angle);       

end
