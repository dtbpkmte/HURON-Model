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
    
    % Prints COP
    calcCOP(r1_ft, l1_ft)

end

% Shutdown ROS Node
rosshutdown;
%}
%--------------EXAMPLE CODE ENDS----------------%

%% 
function cop_x = calcCOP(r1_ft, l1_ft)
%CALCCOP Returns COP_x in meters
%   Outputs:
%   cop_x: x coordinate of COP
%   Inputs:
%   r1_ft: Wrench message from right sensor
%   l1_ft: Wrench message from left sensor

    % Vertical distance from the load cell to bottom of the foot
    d = 0.0983224252792114;
    % Position of left sensor
    p1 = [0; 0.0775; d];
    % Position of right sensor
    p2 = [0; -0.0775; d];

    tau_R = [
        r1_ft.Wrench.Torque.X, ...
        r1_ft.Wrench.Torque.Y, ...
        r1_ft.Wrench.Torque.Z
    ];
    tau_L = [
        l1_ft.Wrench.Torque.X, ...
        l1_ft.Wrench.Torque.Y, ...
        l1_ft.Wrench.Torque.Z
    ];
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

    cop_x = (tau_L(2) + d*f_L(3) + tau_R(2) + d*f_R(3)) ./ (f_L(1) + f_R(1));
end

