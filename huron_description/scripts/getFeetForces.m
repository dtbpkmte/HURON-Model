%% Usage:
%   - See example code below
%   - No need to copy the function, just put this readCOP.m file in the
%   same folder as your code

%% Example code:
%--------------EXAMPLE CODE BEGINS----------------%
%{
% ROS Setup
rosshutdown;
rosinit;

% FT sensor subscribers
r1_ft_sensor = rossubscriber('/huron/sensor/r1_ft_sensor');
l1_ft_sensor = rossubscriber('/huron/sensor/l1_ft_sensor');

t = 0; % time
while t < 15 % loop for 15 seconds (sim time)
    % Reading time
    t = rostime('now');
    t = t.Sec + t.Nsec/1e9;

    % read the force sensor readings right foot
    r1_ft = receive(r1_ft_sensor);
    % read the force sensor readings left foot
    l1_ft = receive(l1_ft_sensor);
    
    % Prints forces
    [F_R, F_L] = getFeetForces(r1_ft, l1_ft)

end

% Shutdown ROS Node
rosshutdown;
%}
%--------------EXAMPLE CODE ENDS----------------%

%% 
function [F_R, F_L] = getFeetForces(r1_ft, l1_ft)
%GETFEETFORCES Returns forces in both feet, x (forward), y, z directions.
%   Outputs:
%       F_R: 2x1, [F_Rx; F_Ry; F_Rz]: Right foot forces
%       F_L: 2x1, [F_Lx; F_Ly; F_Lz]: Left foot forces
%   Inputs:
%   r1_ft: Wrench message from right sensor
%   l1_ft: Wrench message from left sensor

    f_R = [
        r1_ft.Wrench.Force.X, ...
        r1_ft.Wrench.Force.Y, ...
        r1_ft.Wrench.Force.Z
    ];
    f_L = [
        l1_ft.Wrench.Force.X, ...
        l1_ft.Wrench.Force.Y, ...
        l1_ft.Wrench.Force.Z
    ];

    F_R = [f_R(3); f_R(2); -f_R(1)];
    F_L = [f_L(3); f_L(2); -f_L(1)];
end

