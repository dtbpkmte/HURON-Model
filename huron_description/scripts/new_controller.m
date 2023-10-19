%syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 I1 I2 I3 theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot g
% syms t theta1(t) theta2(t) theta3(t)
clear; close all;
rosshutdown;
% ROS Setup
rosinit;



j1_effort = rospublisher('/huron/l_ankle_pitch_effort/command');
j2_effort = rospublisher('/huron/l_ankle_roll_effort/command');
j3_effort = rospublisher('/huron/l_knee_pitch_effort/command');
j4_effort = rospublisher('/huron/l_hip_pitch_effort/command');
j5_effort = rospublisher('/huron/l_hip_roll_effort/command');

j6_effort = rospublisher('/huron/r_ankle_pitch_effort/command');
j7_effort = rospublisher('/huron/r_ankle_roll_effort/command');
j8_effort = rospublisher('/huron/r_knee_pitch_effort/command');
j9_effort = rospublisher('/huron/r_hip_pitch_effort/command');
j10_effort= rospublisher('/huron/r_hip_roll_effort/command');

JointStates = rossubscriber('/huron/joint_states');

r1_ft_sensor = rossubscriber('/huron/sensor/r1_ft_sensor');
l1_ft_sensor = rossubscriber('/huron/sensor/l1_ft_sensor');

 
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau3 = rosmessage(j3_effort);
tau4 = rosmessage(j4_effort);
tau5 = rosmessage(j5_effort);
tau6 = rosmessage(j6_effort);
tau7 = rosmessage(j7_effort);
tau8 = rosmessage(j8_effort);
tau9 = rosmessage(j9_effort);
tau10 = rosmessage(j10_effort);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'huron_description';
req.UrdfParamName = 'robot_description';
req.JointNames = {'l_ankle_pitch_joint','l_ankle_roll_joint' ,'l_hip_pitch_joint','l_hip_roll_joint', 'l_hip_yaw_joint' , 'l_knee_pitch_joint',  'r_ankle_pitch_joint','r_ankle_roll_joint' ,'r_hip_pitch_joint','r_hip_roll_joint', 'r_hip_yaw_joint' , 'r_knee_pitch_joint' };
req.JointPositions = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0) deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];
resp = call(client,req,'Timeout',3);
% tic;
t = 0;

% Varabiles for plotting
Theta_COM=[];
W_velocity_COM=[];
COP=[];
Time=[];

% Main Loop
count = 1;

data_size = 10000;
tau_plot = zeros(3, data_size);
tau_components = zeros(6, data_size);
theta_plot = zeros(3, data_size);
timeStamps = zeros(1, data_size);
com_plot = zeros(1, data_size);
x_com_dot_plot = zeros(1, data_size);

sum_theta_error = [0; 0; 0];
sum_theta_error_plot = zeros(3, data_size);

tOld = rostime('now');
tOld = tOld.Sec + tOld.Nsec/1e9;
while(t <= 15)
% t = toc;
t = rostime('now');
t = t.Sec + t.Nsec/1e9;
 
% Read the Joint States Ankle, Knee, Hip
jointData = receive(JointStates);

% Joint Angles
ankle_pitch_theta=wrapToPi(jointData.Position(1) );
knee_pitch_theta=wrapToPi(jointData.Position(6) );
hip_pitch_theta=wrapToPi(jointData.Position(3) );
 
% Joint Velocities
ankle_pitch_theta_dot=jointData.Velocity(1) ;
knee_pitch_theta_dot=jointData.Velocity(6) ;
hip_pitch_theta_dot=jointData.Velocity(3) ;
 

% desired position, velocity and accelaration of location of the com
theta1_d=0;
theta2_d=0;
theta3_d=0;

theta1_dot_d=0;
theta2_dot_d=0;
theta3_dot_d=0;

theta1_dddot=0;
theta2_dddot=0;
theta3_dddot=0;

x_com_d= 0;
x_com_ddot = 0;
x_com_dddot = 0;

% theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot
theta1=ankle_pitch_theta;
theta2=knee_pitch_theta;
theta3=hip_pitch_theta;
theta1_dot=ankle_pitch_theta_dot;
theta2_dot=knee_pitch_theta_dot;
theta3_dot=hip_pitch_theta_dot;


%% COP Calculation from Force/Torque Sensors at the ankle joint.
% read the force sensor readings right foot
% r1_ft = receive(r1_ft_sensor);
% read the force sensor readings left foot
% l1_ft = receive(l1_ft_sensor);
% Prints COP

% COP(end+1)=calcCOP(r1_ft, l1_ft) ;
%% end of COP calculation from Force/Torque sensors at the ankle joint.


%% EOM of 3 DOF model
 m1=5.9117; m2=4.2554; m3=10.19329; lc1=0.15149; lc2=0.24517 ; lc3=0.1585; l1=0.3715; l2=0.49478; l3=0.32662; g=9.81; I1=0.0222 ; I2=0.01009 ;I3=0.0219 ; % mass in kg, length in meter
 q1 = theta1;
 q2 = theta2;
 q3 = theta3;
 q_dot1 = theta1_dot;
 q_dot2 = theta2_dot;
 q_dot3 = theta3_dot;
 r1 = lc1;
 r2 = lc2;
 r3 = lc3;
 
 M = [I1 + I2 + I3 + l1^2*m2 + l1^2*m3 + l2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + 2*l1*m3*r3*cos(q2 + q3) + 2*l1*l2*m3*cos(q2) + 2*l1*m2*r2*cos(q2) + 2*l2*m3*r3*cos(q3), m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3);
m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, m3*l2^2 + 2*m3*cos(q3)*l2*r3 + m2*r2^2 + m3*r3^2 + I2 + I3, m3*r3^2 + l2*m3*cos(q3)*r3 + I3;
I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3), m3*r3^2 + l2*m3*cos(q3)*r3 + I3, m3*r3^2 + I3];

 C = [- l1*m3*q_dot2^2*r3*sin(q2 + q3) - l1*m3*q_dot3^2*r3*sin(q2 + q3) - l1*l2*m3*q_dot2^2*sin(q2) - l1*m2*q_dot2^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l1*m3*q_dot1*q_dot2*r3*sin(q2 + q3) - 2*l1*m3*q_dot1*q_dot3*r3*sin(q2 + q3) - 2*l1*m3*q_dot2*q_dot3*r3*sin(q2 + q3) - 2*l1*l2*m3*q_dot1*q_dot2*sin(q2) - 2*l1*m2*q_dot1*q_dot2*r2*sin(q2) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    l1*m3*q_dot1^2*r3*sin(q2 + q3) + l1*l2*m3*q_dot1^2*sin(q2) + l1*m2*q_dot1^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    l1*m3*q_dot1^2*r3*sin(q2 + q3) + l2*m3*q_dot1^2*r3*sin(q3) + l2*m3*q_dot2^2*r3*sin(q3) + 2*l2*m3*q_dot1*q_dot2*r3*sin(q3)];

 G = [- g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*l1*m2*sin(q1) - g*l1*m3*sin(q1) - g*m1*r1*sin(q1) - g*m3*r3*sin(q1 + q2 + q3);
    - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*m3*r3*sin(q1 + q2 + q3);
    -g*m3*r3*sin(q1 + q2 + q3)];

 N = C + G; 
%% Center of Mass COM calculation.
X_COM= -1*((lc1*sin(theta1))*m1 + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2 + (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3) / (m1+m2+m3); % Center of Mass position in x_direction
X_dot_COM= -1* (m1*(theta1_dot*lc1*cos(theta1)) + m2*(theta1_dot*l1*cos(theta1) +(theta1_dot+theta2_dot)*lc2*cos(theta1+theta2)) + m3*(theta1_dot*l1*cos(theta1)+(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)+(theta1_dot+theta2_dot+theta3_dot)*lc3*cos(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in x_direction
J_X_COM= -1* [(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1))/(m1 + m2 + m3), (m3*(l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3)) + lc2*m2*cos(theta1 + theta2))/(m1 + m2 + m3), (lc3*m3*cos(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of X-COM Jx
J_X_COM_dot=-1*[(-m1*theta1_dot*lc1*sin(theta1) + m2*(-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) )  + m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2)-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3) ,( m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2) -lc3*(theta1_dot+theta2_dot+theta3_dot)*sin(theta1+theta2+theta3) ) -lc2*(theta1_dot+theta2_dot)*m2*sin(theta1+theta2) )/(m1+m2+m3) ,(-lc3*(theta1_dot+theta2_dot+theta3_dot)*m3*sin(theta1+theta2+theta3) )/(m1+m2+m3)]; % Time Derivative of Jacobian Matrix
Pseudo_J_X_COM=pinv(J_X_COM); % Pseudo Inverse of J_X_COM

Z_COM=( (lc1*cos(theta1))*m1 + (l1*cos(theta1)+lc2*cos(theta1+theta2))*m2 + (l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); % Center of Mass position in z_direction
Z_dot_COM=(  -m1*(theta1_dot*lc1*sin(theta1)) + m2*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) ) + m3*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*l2*sin(theta1+theta2) - (theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in z_direction 
J_Z_COM=[(m3*(-l2*sin(theta1 + theta2) - l1*sin(theta1) - lc3*sin(theta1 + theta2 + theta3)) + m2*(-lc2*sin(theta1 + theta2) - l1*sin(theta1)) - lc1*m1*sin(theta1))/(m1 + m2 + m3), (m3*(-l2*sin(theta1 + theta2) - lc3*sin(theta1 + theta2 + theta3)) - lc2*m2*sin(theta1 + theta2))/(m1 + m2 + m3), (-lc3*m3*sin(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of Z-COM Jz
 
J_COM=[J_X_COM ; J_Z_COM]; % Linear part of Jacobian matrix of COM
%..........................................................................
J_W_COM=[1 , 1 ,1 ];
J_total_COM=[J_X_COM ; J_W_COM] ; % linear and angular part of Jacobian of COM
J_total_COM_dot=[ J_X_COM_dot ; 0 0 0]; % time derivative of total Jacobian of COM
J_total_COM_pseduo=pinv(J_total_COM);  % pseduo inverse of the total Jacobian of COM
 %%  End of Center of Mass COM calculation.

 %% Calculation of Center of mass COM angular position and velocity
% X_position_COM=( (lc1*sin(-theta1(t)))*m1 + (l1*sin(-theta1(t))+lc2*sin(-theta1(t)-theta2(t)))*m2 + (l1*sin(-theta1(t))+l2*sin(-theta1(t)-theta2(t))+lc3*sin(-theta1(t)-theta2(t)-theta3(t)))*m3  ) / (m1+m2+m3) ; % Center of Mass position in x_direction
% Z_position_COM=( (lc1*cos(-theta1(t)))*m1 + (l1*cos(-theta1(t))+lc2*cos(-theta1(t)-theta2(t)))*m2 + (l1*cos(-theta1(t))+l2*cos(-theta1(t)-theta2(t))+lc3*cos(-theta1(t)-theta2(t)-theta3(t)))*m3  ) / (m1+m2+m3);   % Center of Mass position in z_direction
% Calculation of Angle and velocity of COM w.r.t ankle joint
% X_COM=( (lc1*sin(-theta1(t)))*m1 + (l1*sin(-theta1(t))+lc2*sin(-theta1(t)-theta2(t)))*m2 + (l1*sin(-theta1(t))+l2*sin(-theta1(t)-theta2(t))+lc3*sin(-theta1(t)-theta2(t)-theta3(t)))*m3  ) / (m1+m2+m3) ; % Center of Mass position in x_direction
% Z_COM=( (lc1*cos(-theta1(t)))*m1 + (l1*cos(-theta1(t))+lc2*cos(-theta1(t)-theta2(t)))*m2 + (l1*cos(-theta1(t))+l2*cos(-theta1(t)-theta2(t))+lc3*cos(-theta1(t)-theta2(t)-theta3(t)))*m3  ) / (m1+m2+m3);   % Center of Mass position in z_direction
%  Angle_COM=atan2( X_COM , Z_COM )  % In rad 
%  angular_velocity_COM=diff(Angle_COM,t)
% Angle_COM=angle((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3) - ((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))*1i)/(m1 + m2 + m3)) ; 
% W_COM=-((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2*((imag((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) + real((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3)))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3))) - ((imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))*(real((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) - imag((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3))))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2))/((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2 + (imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2);

Angle_COM=-(theta1+theta2+theta3);
W_COM=-(theta1_dot+theta2_dot+theta3_dot);
 %% End of calculation of center of mass angular position and velocity

 %% Coupled Linear and Angular momentum controller using PD controller
 kpx=1  ; kdx=2 ; kptheta=1 ; kdtheta=0.5;
 P_COM_ddot= -kpx * (X_COM) - kdx * (X_dot_COM);
 alpha_COM= -kptheta * (Angle_COM) - kdtheta * (W_COM) ;

 theta_ddot_reference=  J_total_COM_pseduo * ( [P_COM_ddot ; alpha_COM] - J_total_COM_dot *[ theta1_dot ; theta2_dot ; theta3_dot ] ) ; 
 Torque_PD_coupled= M * theta_ddot_reference + N ; % torque generated by coupled momentum sith PD controller
 %% end of linear and angular momentum controller using PD controller

  %% End of calculation of center of mass angular position and velocity


%% Coupled Linear and Angular momentum controller using SMC controller
lambda = [2 0; 0 2]; k1 = [0.3 0; 0 0.3]; k2 = [0.2 0; 0 0.2]; phi = 0.001; % F = 7000 at z = 0.1      
error = [X_COM; Angle_COM];
error_dot = [X_dot_COM; W_COM];
theta_dot = [theta1_dot; theta2_dot; theta3_dot];
s = error_dot + lambda*error;
if norm(s) >= phi
    sat = sign(s);
else
    sat = s / phi;
end
tau_coupled_smc= N - M*J_total_COM_pseduo*(k1*s + k2*sat + J_total_COM_dot*theta_dot + lambda*error_dot);
%% end of linear and angular momentum controller using SMC controller


%% Sliding mode control(Linear sliding surface + linear reaching law).
% s = edot + lambda*e, sdot = -K1*s -K2*sat(s)  
%lambda = 0.1; k1 = 0.5; k2 = 0.05; phi = 0.01; % F = 5000 at z = 0   
lambda = 2; k1 = 1; k2 = 0.12; phi = 0.001; % F = 5000 at z = 0.1   
error = X_COM - x_com_d;
error_dot = X_dot_COM - x_com_ddot;
theta_dot = [theta1_dot; theta2_dot; theta3_dot];
s = error_dot + lambda*error;
if norm(s) >= phi
    sat = sign(s);
else
    sat = s / phi;
end
tau = N - M*Pseudo_J_X_COM*(k1*s + k2*sat + J_X_COM_dot*theta_dot - x_com_d + lambda*error_dot);
%% End of Sliding mode control(Linear sliding surface + linear reaching law).

%% adding impedance model at the center of mass of the upper link
P_COM_upper_link=[ l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3) ;  l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3) ] ;
X_COM_upper=l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3) ;
Z_COM_upper=l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3);
X_COM_upper_dot= theta1_dot*l1*cos(theta1)+(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)+(theta1_dot+theta2_dot+theta3_dot)*lc3*cos(theta1+theta2+theta3) ;
J=[l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3),   l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3),  lc3*cos(theta1 + theta2 + theta3) ; - l2*sin(theta1 + theta2) - l1*sin(theta1) - lc3*sin(theta1 + theta2 + theta3), - l2*sin(theta1 + theta2) - lc3*sin(theta1 + theta2 + theta3), -lc3*sin(theta1 + theta2 + theta3)];
FX=-0.1*X_COM_upper-10*X_COM_upper_dot; % Gains work with TCOM CTC controller
FY=m3*9.8;
Torque_Impedance_at_hip=transpose(J) *[FX ;FY];
%% end of adding impedance model at the center of mass of the upper link

%% Addition of Controllers
% T=tau+Torque_Impedance_at_hip; % SMC from Abizer + VMC at upper link
T= Torque_PD_coupled ;        % Torque generated from coupled momentum using PD controller
% T=tau_coupled_smc ; % Torque generated from coupled momentum using SMC controller


if T(1) >= 30
    T(1) = 30;
end
if T(1) <= -11.76
    T(1) = -11.76;
end

T_ankle = T(1);
T_knee = T(2);
T_hip = T(3);





%Store and print values at an interval of 0.1 seconds
if ((t - tOld) >= 0.1 || count == 1)
    tau_plot(1, count) = T_ankle;
    tau_plot(2, count) = T_knee;
    tau_plot(3, count) = T_hip;
    theta_plot(1,count) = theta1;
    theta_plot(2,count) = theta2;
    theta_plot(3,count) = theta3;
    tOld = t; 
    timeStamps(1,count) = t;
    com_plot(1,count) = X_COM; 
    count = count + 1;
    fprintf("error is: ");
    fprintf("%f %f %f\n", error);
    fprintf("theta is: ");
    fprintf("%f %f %f\n", theta1, theta2, theta3);
    % fprintf("s is: ");
    % fprintf("%f %f %f\n", s);
    fprintf("torque is: ");
    fprintf("%f %f %f\n", T_ankle, T_knee, T_hip);
end

% Publish Torque to Joints
  tau1.Data =T_ankle; % left ankle pitch
  tau2.Data =0; % left ankle roll
  tau3.Data =T_knee; % left knee pitch
  tau4.Data = T_hip ; % left hip pitch
%   tau5.Data =0;  % left hip roll
  tau6.Data =T_ankle ; % right ankle pitch
  tau7.Data =0; % right ankle roll
  tau8.Data =T_knee; % right knee pitch
  tau9.Data =T_hip; % right hip pitch 
%   tau10.Data = 0 ; % right hip roll
  send(j1_effort,tau1);
  send(j2_effort,tau2);
  send(j3_effort,tau3);
  send(j4_effort,tau4);
%   send(j5_effort,tau5);
  send(j6_effort,tau6);
  send(j7_effort,tau7);
  send(j8_effort,tau8);
  send(j9_effort,tau9);

 
  Theta_COM(end+1)=Angle_COM;
  W_velocity_COM(end+1)=W_COM;
  Time(end+1)=t;
end
 Theta_COM';
 W_velocity_COM';
 Time';
 COP' ; 
 



 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tau1.Data = 0;
tau2.Data = 0;
tau3.Data = 0;
tau4.Data = 0;
tau5.Data = 0;
tau6.Data = 0;
tau7.Data = 0;
tau8.Data = 0;
tau9.Data = 0;
tau10.Data =0;
send(j1_effort,tau1);
send(j2_effort,tau2);
send(j3_effort,tau3);
send(j4_effort,tau4);
send(j5_effort,tau5);
send(j6_effort,tau6);
send(j7_effort,tau7);
send(j8_effort,tau8);
send(j9_effort,tau9);
send(j10_effort,tau10);
% disconnect from roscore
rosshutdown;

%% plot the torque and theta
subplot(4,3,1);
plot(timeStamps, tau_plot(1,:),"Color",'r');
legend("Torque of ankle pitch")
subplot(4,3,2);
plot(timeStamps, tau_plot(2,:),"Color",'c');
legend("Torque of knee pitch")
subplot(4,3,3);
plot(timeStamps, tau_plot(3,:),"Color",'b');
legend("Torque of hip pitch")
subplot(4,3,4);
plot(timeStamps, theta_plot(1,:),"Color",'r');
legend("ankle pitch")
subplot(4,3,5);
plot(timeStamps, theta_plot(2,:),"Color",'c');
legend("knee pitch")
subplot(4,3,6);
plot(timeStamps, theta_plot(3,:),"Color",'b');
legend("hip pitch")
subplot(4,3,7);
plot(timeStamps, com_plot,"Color",'m');
legend("com")
 

figure;
subplot(1,2,1)
plot(Time,Theta_COM )
title('COM Angular Position in rad')
subplot(1,2,2)
plot(Time,W_velocity_COM)
title('COM Angular Velocity in rad/sec')

figure;
plot(Time,COP)
title('COP Position in m ')



function cop_x = calcCOP(r1_ft, l1_ft)
%READCOP Returns COP_x in meters
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