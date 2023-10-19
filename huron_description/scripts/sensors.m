clear; close; clc;
rosshutdown;
% ROS Setup
rosinit;

joint_effort_pub = rospublisher('/huron/joint_group_effort_controller/command');

JointStates = rossubscriber('/huron/joint_states');

Imu=rossubscriber('/huron/sensor/imu');

r1_ft_sensor = rossubscriber('/huron/sensor/r1_ft_sensor');
l1_ft_sensor = rossubscriber('/huron/sensor/l1_ft_sensor');
% contact_sensor=rossubscriber('/fixed_jaw_contact_sensor_state');

% Vertical distance from the load cell to bottom of the foot
d = 0.0983224252792114;
% Position of left sensor
p1 = [0; 0.0775; d];
% Position of right sensor
p2 = [0; -0.0775; d];

% x_ZMP limits
x_ZMP_lims = [-50e-3, 157.88e-3]; % m

tau = rosmessage(joint_effort_pub);
tau.Data = [0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0];

send(joint_effort_pub,tau);

client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'huron_description';
req.UrdfParamName = 'robot_description';
req.JointNames = {'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint', 'r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint', 'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint'  };
req.JointPositions = [deg2rad(0), deg2rad(0) ,deg2rad(0), deg2rad(0) ,deg2rad(0), deg2rad(0) ,deg2rad(0), deg2rad(0)   ,deg2rad(0), deg2rad(0)  ] ;
resp = call(client,req,'Timeout',3);

prealloc_size = 10000;
t = 0;
i = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = zeros(prealloc_size, 1); % sec
tau_L = zeros(prealloc_size, 3); % [tau_ix tau_iy tau_iz]
f_L = zeros(prealloc_size, 3); % [f_ix f_iy f_iz]
tau_R = zeros(prealloc_size, 3); % [tau_ix tau_iy tau_iz]
f_R = zeros(prealloc_size, 3); % [f_ix f_iy f_iz]

p_ZMP_x = zeros(prealloc_size, 1);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while t < 15 && i <= prealloc_size
    t = rostime('now');
    t = t.Sec + t.Nsec/1e9;
    time(i) = t;

    %read the force sensor readings right foot
    r1_ft = receive(r1_ft_sensor);

    %read the force sensor readings left foot
    l1_ft = receive(l1_ft_sensor);
    % 
    % tau_R(i, :) = [
    %     r1_force_sensor.Wrench.Torque.X, ...
    %     r1_force_sensor.Wrench.Torque.Y, ...
    %     r1_force_sensor.Wrench.Torque.Z
    % ];
    % tau_L(i, :) = [
    %     l1_force_sensor.Wrench.Torque.X, ...
    %     l1_force_sensor.Wrench.Torque.Y, ...
    %     l1_force_sensor.Wrench.Torque.Z
    % ];
    % f_R(i, :) = [
    %     r1_force_sensor.Wrench.Force.X, ...
    %     r1_force_sensor.Wrench.Force.Y, ...
    %     r1_force_sensor.Wrench.Force.Z
    % ];
    % f_L(i, :) = [
    %     l1_force_sensor.Wrench.Force.X, ...
    %     l1_force_sensor.Wrench.Force.Y, ...
    %     l1_force_sensor.Wrench.Force.Z
    % ];

    p_ZMP_x(i) = readCOP(r1_ft, l1_ft);


    send(joint_effort_pub, tau);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i = i + 1;
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

% Trim out unfilled data
time = time(1:i-1);
% tau_L = tau_L(1:i-1, :);
% tau_R = tau_R(1:i-1, :);
% f_L = f_L(1:i-1, :);
% f_R = f_R(1:i-1, :);
p_ZMP_x = p_ZMP_x(1:i-1, :);

% Add the equations to calculate the COP 
% figure;
% subplot(1, 2, 1)
% plot(time, tau_L)
% title('Left foot Torque')
% subplot(1, 2, 2)
% plot(time, f_L)
% title('Left foot Force')
% 
% figure;
% subplot(1, 2, 1)
% plot(time, tau_R)
% title('Right foot Torque')
% subplot(1, 2, 2)
% plot(time, f_R)
% title('Right foot Force')

% Check correctness of force sensors by mass
% robot_mass = 37.2063;
% g = 9.81;
% figure;
% plot(time, robot_mass * g * ones(size(time))); % correct value
% hold on
% f_total = f_L(:, 1) + f_R(:, 1);
% plot(time, f_total);
% hold off

% ZMP plot
% p_Ly = (f_Lz * l_force_sensor_pos(:, 2)) ./ sum(f_Lz, 2);
% p_Ry = (f_Rz * r_force_sensor_pos(:, 2)) ./ sum(f_Rz, 2);
% p_ZMP_x = (tau_L(:, 2) + d*f_L(:, 3) + tau_R(:, 2) + d*f_R(:, 3)) ./ (f_L(:, 1) + f_R(:, 1));
figure;
plot(time, p_ZMP_x);
title('x_ZMP raw')
xlabel('time (s)'); ylabel('x (m)');
hold on
plot(time, x_ZMP_lims(1) * ones(size(time)))
plot(time, x_ZMP_lims(2) * ones(size(time)))
hold off

% Filtered ZMP plot (LPF)
p_ZMP_x_filtered = lowpass(p_ZMP_x, 10, 100);
figure;
plot(time, p_ZMP_x_filtered);
title('x_ZMP LPF')
xlabel('time (s)'); ylabel('x (m)');
hold on
plot(time, x_ZMP_lims(1) * ones(size(time)))
plot(time, x_ZMP_lims(2) * ones(size(time)))
hold off

% Median filter, 45th order
p_ZMP_med_filt = medfilt1(p_ZMP_x, 45);
figure;
plot(time, p_ZMP_med_filt);
title('x_ZMP med')
xlabel('time (s)'); ylabel('x (m)');
hold on
plot(time, x_ZMP_lims(1) * ones(size(time)))
plot(time, x_ZMP_lims(2) * ones(size(time)))
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
send(joint_effort_pub, tau);
 
% disconnect from roscore
rosshutdown;
% plot the trajectories
