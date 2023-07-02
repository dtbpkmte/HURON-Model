%syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 I1 I2 I3 theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot g
clear; close all;
rosshutdown;
% ROS Setup
rosinit;

% effort_controller = rospublisher('/huron/joint_group_effort_controller/command');

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
% tau_msg = rosmessage(effort_controller);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'huron_description';
req.UrdfParamName = 'robot_description';
req.JointNames = {'l_ankle_pitch_joint','l_ankle_roll_joint' ,'l_hip_pitch_joint','l_hip_roll_joint', 'l_hip_yaw_joint' , 'l_knee_pitch_joint',  'r_ankle_pitch_joint','r_ankle_roll_joint' ,'r_hip_pitch_joint','r_hip_roll_joint', 'r_hip_yaw_joint' , 'r_knee_pitch_joint' };
req.JointPositions = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0) deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];
resp = call(client,req,'Timeout',3);
% tic;
t = 0;

 

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
 
J_COM=[J_X_COM ; J_Z_COM];
 %%  End of Center of Mass COM calculation.

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
q_SMC_dotdot = -Pseudo_J_X_COM*(k1*s + k2*sat + J_X_COM_dot*theta_dot - x_com_d + lambda*error_dot);
tau = N + M*q_SMC_dotdot;

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

%% Posture correction
theta_d = [theta1_d; theta2_d; theta3_d];
theta = [theta1; theta2; theta3];
theta_dot = [theta1_dot; theta2_dot; theta3_dot];

Kp_posture = diag([100 100 100]); % Gain matrix 
Ki_posture = diag([50 50 50]);
Kd_posture = diag([1 1 1]);

%% Posture correction Jacobian - x2 = theta_d - theta
% J2 = -eye(3);
% J2_dot = 0;

%% Posture correction - x2 = 0.5*sum(theta.^2)
J2 = theta';
J2_dot = theta_dot';

%% Sadeghian 2011 - acceleration level
% N1 = eye(3) - Pseudo_J_X_COM*J_X_COM;
% J2_bar = J2 * N1;
% J2_bar_pinv = pinv(J2_bar);
% 
% % Integral
% epsilon = 1e-2;
% theta_error = (theta_d - theta);
% if norm(theta_error) <= epsilon
%     sum_theta_error = zeros(3, 1);
% else
%     sum_theta_error = sum_theta_error + theta_error*(t - tOld);
% end
% 
% % T_posture = M*(eye(3) - Pseudo_J_X_COM*J_X_COM)*Ki_posture*sum_theta_error;
% % T_posture = M*(eye(3) - Pseudo_J_X_COM*J_X_COM)*(K_posture*theta_error + Ki_posture*sum_theta_error);
% 
% % Control
% % x2_dotdot = Kp_posture*(theta_d - theta);
% x2_dotdot = J2_dot*theta_dot + J2*(Kp_posture*(theta_d - theta) + Kd_posture*(-theta_dot));
% 
% q2_dotdot = N1*J2_bar_pinv*(x2_dotdot - J2_dot*theta_dot - J2*q_SMC_dotdot);
% T_posture = M*q2_dotdot;

%% Khatib 2004 - torque level
% J1 = J_X_COM;
% M_inv = inv(M);
% J1_bar = M_inv*J1'*pinv(J1*M_inv*J1');
% N1_T = eye(3) - J1'*J1_bar;
% J2_1 = J2 * N1_T';
% Lambda_2_1 = pinv(J2_1*M_inv*J2_1');
% J2_1_bar = M_inv*J2_1'*Lambda_2_1;

%% Nakanishi 2008


%% 3.2.1. Hsu, eqn. 36
% ksi_2 = J_X_COM_dot'*Pseudo_J_X_COM'*(theta_dot - ksi_1) + ...
        % ksi_1_dot - ...
        % K_n*(eye(3) - Pseudo_J_X_COM*J_X_COM)*(theta_dot - ksi_1);

%% 3.2.2 Simplified with M eq.42
% 
%Gradient 
% Kw = diag([1 1 1]);
% del_g = Kw * theta;
% del_g_dot = Kw * theta_dot;
% 
% %%Ksi
% a = 100;
% K_n = diag([1 1 1]);
% 
% ksi_1 = -a*del_g;
% ksi_1_dot = -a*del_g_dot;
% 
% K_q_d = diag([0 0 0]);
% ksi_2_simp = -K_q_d*theta_dot + ksi_1; 
%% 3.2.3 Simplified without M eqn.43
% %Gradient 
Kw = diag([1 1 1]);
del_g = Kw * theta;
del_g_dot = Kw * theta_dot;

%%Ksi
a = 100;
K_n = diag([1 1 1]);

ksi_1 = -a*del_g;
ksi_1_dot = -a*del_g_dot;

K_q_d = diag([1 1 1]);
ksi_2_simp = -K_q_d*theta_dot + ksi_1;

%% Dynamically consistent
% M_inv = inv(M);
% weighted_J_X_COM = M_inv*J_X_COM'*inv(J_X_COM*M_inv*J_X_COM');
% tau_null = -K_q_d*theta_dot + ksi_1;
% T_posture = (eye(3) - J_X_COM'*weighted_J_X_COM')*tau_null;
% q_SMC_dotdot = -weighted_J_X_COM*(k1*s + k2*sat + J_X_COM_dot*theta_dot - x_com_d + lambda*error_dot);
% tau = N + M*q_SMC_dotdot;
%% Posture torque
Phi_N = (eye(3) - Pseudo_J_X_COM*J_X_COM)*ksi_2_simp;
T_posture = Phi_N;

%% End of Posture correction

% Pseudo_J_X_COM%% Addition of Controllers
% T=tau+Torque_Impedance_at_hip + T_posture; % SMC from Abizer + VMC at upper link
% T=tau; % SMC from Abizer
T = tau + T_posture; % SMC + posture correction
% T = T_posture; % SMC + posture correction
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
% if ((t - tOld) >= 0.1 || count == 1)
    tau_plot(1, count) = T_ankle;
    tau_plot(2, count) = T_knee;
    tau_plot(3, count) = T_hip;
    theta_plot(1,count) = theta1;
    theta_plot(2,count) = theta2;
    theta_plot(3,count) = theta3;
    tOld = t; 
    timeStamps(1,count) = t;
    com_plot(1,count) = X_COM; 
    
    % Torque contributed by SMC
    % Torque contributed by Posture correction
    tau_components(:, count) = [tau; T_posture];

    sum_theta_error_plot(:, count) = sum_theta_error;

    x_com_dot_plot(1, count) = X_dot_COM;

    count = count + 1;

    % fprintf("error is: ");
    % fprintf("%f %f %f\n", error);
    % fprintf("theta is: ");
    % fprintf("%f %f %f\n", theta1, theta2, theta3);
    % % fprintf("s is: ");
    % % fprintf("%f %f %f\n", s);
    % fprintf("torque is: ");
    % fprintf("%f %f %f\n", T_ankle, T_knee, T_hip);
% end

% Publish Torque to Joints
  tau1.Data =T_ankle; % left ankle pitch
  tau2.Data =0; % left ankle roll
  tau3.Data =T_knee; % left knee pitch
  tau4.Data = T_hip ; % left hip pitch
  tau5.Data =0;  % left hip roll
  tau6.Data =T_ankle ; % right ankle pitch
  tau7.Data =0; % right ankle roll
  tau8.Data =T_knee; % right knee pitch
  tau9.Data =T_hip; % right hip pitch 
  tau10.Data = 0 ; % right hip roll
    % tau_msg.Data = [T_ankle, 0, T_hip, 0, 0, T_knee, ...
    %                 T_ankle, 0, T_hip, 0, 0, T_knee];
  send(j1_effort,tau1);
  send(j2_effort,tau2);
  send(j3_effort,tau3);
  send(j4_effort,tau4);
%   send(j5_effort,tau5);
  send(j6_effort,tau6);
  send(j7_effort,tau7);
  send(j8_effort,tau8);
  send(j9_effort,tau9);
    % send(effort_controller, tau_msg)
  

 
end
 
tau_plot = tau_plot(:, 1:count-1);
tau_components = tau_components(:, 1:count-1);
theta_plot = theta_plot(:, 1:count-1);
timeStamps = timeStamps(:, 1:count-1);
com_plot = com_plot(:, 1:count-1);
sum_theta_error_plot = sum_theta_error_plot(:, 1:count-1);
x_com_dot_plot = x_com_dot_plot(:, 1:count-1);

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
% tau_msg.Data = zeros(1, 12);
% send(effort_controller, tau_msg);
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
subplot(4,3,10);
plot(timeStamps, com_plot,"Color",'m');
legend("com")
subplot(4,3,7);
plot(timeStamps, tau_components(1, :),"Color",'m');
hold on
plot(timeStamps, tau_components(4, :),"Color",'g');
legend("T_SMC ankle", "T_posture ankle")
hold off
subplot(4,3,8);
plot(timeStamps, tau_components(2, :),"Color",'m');
hold on
plot(timeStamps, tau_components(5, :),"Color",'g');
legend("T_SMC knee", "T_posture knee")
hold off
subplot(4,3,9);
plot(timeStamps, tau_components(3, :),"Color",'m');
hold on
plot(timeStamps, tau_components(6, :),"Color",'g');
legend("T_SMC hip", "T_posture hip")
hold off
subplot(4,3,11);
plot(timeStamps, sum_theta_error_plot);
% TODO: plot norm(theta_error)
subplot(4,3,12);
plot(timeStamps, x_com_dot_plot);

 
