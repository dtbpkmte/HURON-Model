%syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 I1 I2 I3 theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot g
% syms t theta1(t) theta2(t) theta3(t)

rosshutdown;

clear; close all;

% ROS Setup
rosinit;



% j1_effort = rospublisher('/huron/l_ankle_pitch_effort/command');
% j2_effort = rospublisher('/huron/l_ankle_roll_effort/command');
% j3_effort = rospublisher('/huron/l_knee_pitch_effort/command');
% j4_effort = rospublisher('/huron/l_hip_pitch_effort/command');
% j5_effort = rospublisher('/huron/l_hip_roll_effort/command');
% 
% j6_effort = rospublisher('/huron/r_ankle_pitch_effort/command');
% j7_effort = rospublisher('/huron/r_ankle_roll_effort/command');
% j8_effort = rospublisher('/huron/r_knee_pitch_effort/command');
% j9_effort = rospublisher('/huron/r_hip_pitch_effort/command');
% j10_effort= rospublisher('/huron/r_hip_roll_effort/command');
effort_controller = rospublisher('/huron/joint_group_effort_controller/command');

JointStates = rossubscriber('/huron/joint_states');

r1_ft_sensor = rossubscriber('/huron/sensor/r1_ft_sensor');
l1_ft_sensor = rossubscriber('/huron/sensor/l1_ft_sensor');

 
% tau1 = rosmessage(j1_effort);
% tau2 = rosmessage(j2_effort);
% tau3 = rosmessage(j3_effort);
% tau4 = rosmessage(j4_effort);
% tau5 = rosmessage(j5_effort);
% tau6 = rosmessage(j6_effort);
% tau7 = rosmessage(j7_effort);
% tau8 = rosmessage(j8_effort);
% tau9 = rosmessage(j9_effort);
% tau10 = rosmessage(j10_effort);
tau_msg = rosmessage(effort_controller);

client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'huron_description';
req.UrdfParamName = 'robot_description';
req.JointNames = {'l_ankle_pitch_joint','l_ankle_roll_joint' ,'l_hip_pitch_joint','l_hip_roll_joint', 'l_hip_yaw_joint' , 'l_knee_pitch_joint',  'r_ankle_pitch_joint','r_ankle_roll_joint' ,'r_hip_pitch_joint','r_hip_roll_joint', 'r_hip_yaw_joint' , 'r_knee_pitch_joint' };
req.JointPositions = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0) deg2rad(0), deg2rad(0) , deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];
resp = call(client,req,'Timeout',3);
% tic;
t = 0;

 
tOld = 0;

% Varabiles for plotting
Theta_COM=[];
W_velocity_COM=[];
Theta_COM_calculated=[];
W_velocity_COM_calculated=[];
Position_COM=[];
Linear_velocity_COM=[];
T_SMC_Linear=[];
T_SMC_Angular=[];
COP=[];
Time=[];
S1_dot=[];
S2_dot=[];
S1=[];
S2=[];
% Main Loop
count = 1;
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
ankle_roll_theta=wrapToPi(jointData.Position(2) )  ;

 
% Joint Velocities
ankle_pitch_theta_dot=jointData.Velocity(1) ;
knee_pitch_theta_dot=jointData.Velocity(6) ;
hip_pitch_theta_dot=jointData.Velocity(3) ;
ankle_roll_theta_dot=jointData.Velocity(2) ;

 
 
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
r1_ft = receive(r1_ft_sensor);
% read the force sensor readings left foot
l1_ft = receive(l1_ft_sensor);
% Prints COP

COP(end+1)=calcCOP(r1_ft, l1_ft) ;
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
Angle_COM=-(theta1+theta2+theta3);
W_COM=-(theta1_dot+theta2_dot+theta3_dot);
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
% Using Angluar position of COM and velocity that I calculated
Angle_COM_calculated=angle((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3) - ((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))*1i)/(m1 + m2 + m3)) ; 
W_COM_calculated=-((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2*((imag((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) + real((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3)))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3))) - ((imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))*(real((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) - imag((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3))))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2))/((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2 + (imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2);
Theta_COM_calculated(end+1)=Angle_COM_calculated;
W_velocity_COM_calculated(end+1)=W_COM_calculated;
%% End of calculation of center of mass angular position and velocity

 %% Coupled Linear and Angular momentum controller using PD controller
 kpx=1  ; kdx=2 ; kptheta=1 ; kdtheta=0.5;
 P_COM_ddot= -kpx * (X_COM) - kdx * (X_dot_COM);
 alpha_COM= -kptheta * (Angle_COM) - kdtheta * (W_COM) ;

 theta_ddot_reference=  J_total_COM_pseduo * ( [P_COM_ddot ; alpha_COM] - J_total_COM_dot *[ theta1_dot ; theta2_dot ; theta3_dot ] ) ; 
 Torque_PD_coupled= M * theta_ddot_reference + N ; % torque generated by coupled momentum sith PD controller
 %% end of linear and angular momentum controller using PD controller


% %% Coupled Linear and Angular momentum controller using SMC controller
% lambda = [1 0; 0 1]; k1 = [0.7 0; 0 0.7]; k2 = [0.02 0; 0 0.02]; phi = 0.01; % F = 7000 at z = 0.1      
% error = [X_COM; Angle_COM];
% error_dot = [X_dot_COM; W_COM];
% theta_dot = [theta1_dot; theta2_dot; theta3_dot];
% s = error_dot + lambda*error;
% if norm(s) >= phi
%     sat = sign(s);
% else
%     sat = s / phi;
% end
% tau_coupled_smc= N - M*J_total_COM_pseduo*(k1*s + k2*sat + J_total_COM_dot*theta_dot + lambda*error_dot);
% %% end of linear and angular momentum controller using SMC controller

%% Our Proposed SMC Controller for both Linear and Angular Momentum
error1=X_COM;       error_dot1=X_dot_COM;
error2=Angle_COM;   error_dot2=W_COM;

theta_dot = [theta1_dot; theta2_dot; theta3_dot];

L1=1.25;    k1=-0.002;    p1=0.001;    c1=0.001;    Q1=0.001;      a1=0.001;         z1=0.1;
L2=1;       k2=-1;        p2=1;        c2=1;        Q2=0.01;     a2=0.001;         z2=1;
 
s1 = error_dot1 + L1*error1; 
s2 = error_dot2 + L2*error2; 

f1= c1 * ( 1 - exp( k1*(abs(s1))^p1 ) );
f2= c2 * ( 1 - exp( k2*(abs(s2))^p2 ) );


Beta1= a1 *   sign(abs(s1)-1);
Beta2= a2 * sign(abs(s2)-1);

% % switching reaching law case
% if (abs(s1)>0.05)
%     s1_dot= -Q1 * ( (abs(s1))^f1 ) *sign(s1) - z1 * (abs(s1))^a1  * s1 ;
% 
% end
% if (abs(s2)>0.15)
%     s2_dot= -Q2 * ( (abs(s2))^f2 ) *sign(s2) - z2 * (abs(s2))^a2  * s2   ; 
% 
% end
% 
% y1=0.01; y2=0.01;
% if (abs(s1)<0.05)
%     s1_dot= -y1*abs(s1)*sign(s1) ;
% 
% end
% if (abs(s2)<0.15)
%     s2_dot= -y2*abs(s2)*sign(s2) ;   
% 
% end
% % end of switching reaching law case
 
% No cases case
s1_dot= -Q1 * ( (abs(s1))^f1 ) *sign(s1) - z1 * (abs(s1))^a1  * s1 ;
s2_dot= -Q2 * ( (abs(s2))^f2 ) *sign(s2) - z2 * (abs(s2))^a2  * s2   ; 
% end of No cases case

 
% case when only s1 is active controlling only COM position
Torque_SMC_Linear= M * Pseudo_J_X_COM * s1_dot  - M * Pseudo_J_X_COM * J_X_COM_dot *[theta1_dot ;theta2_dot ;theta3_dot] -M * Pseudo_J_X_COM * (L1 * error_dot1)+ N ;
%  end of case when only s1 is active controlling only COM position

% case when only s2 is active controlling only the angular motion of COM
Torque_SMC_Angular= M * pinv([1 , 1 , 1]) * s2_dot   -M * pinv([1 , 1 , 1])* (L2 * error_dot2)+ N ;
% end of case when only s2 is active controlling only the angular motion of COM

% case when both s1 and s2 are active
Torque_SMC= M * J_total_COM_pseduo * [s1_dot ; s2_dot] - M * J_total_COM_pseduo * J_total_COM_dot *[theta1_dot ;theta2_dot ;theta3_dot] -M * J_total_COM_pseduo * [L1 * error_dot1 ; L2 * error_dot2]+ N ;
% end of both s1 and s2 are active

S1_dot(end+1)=s1_dot;
S2_dot(end+1)=s2_dot;
S1(end+1)=s1;
S2(end+1)=s2;
T_SMC_Linear(:,end+1)=Torque_SMC_Linear;
T_SMC_Angular(:,end+1)=Torque_SMC_Angular;

%% End of our proposed SMC Controller for both Linear and Angular Momentum


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
FX=-10*X_COM_upper-10*X_COM_upper_dot; % Gains work with TCOM CTC controller
FY=m3*9.8;
Torque_Impedance_at_hip=transpose(J) *[FX ;FY];
%% end of adding impedance model at the center of mass of the upper link


%% Ankle Roll joint controller
A_matrix=[0 1 ; 9.81/0.6878 0 ]; % the A matrix of a single inverted pendulum model for the ankle roll control
B_matrix=[ 0 ; 1/(3.5*0.6878^2)];
C_matrix=[1 0 ; 0 1];
D_matrix=[0 ; 0];
Q_matrix=[2 0 ; 0 2];
R_matrix=1;
K_matrix=lqr(A_matrix,B_matrix,Q_matrix,R_matrix);
poles_ankle_roll=[-10 -30];
k_pole_ankle_roll=place(A_matrix,B_matrix,poles_ankle_roll);
% T_roll_ankle=-K_matrix*[l_ankle_roll_theta ;l_ankle_roll_theta_dot ];% LQR CASE
T_roll_ankle=-k_pole_ankle_roll*[ankle_roll_theta ;ankle_roll_theta_dot ];% pole placement case
%% end of Ankle Roll Joint Controller

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
% %Gradient 
% Kw = diag([1 1 1]);
% del_g = Kw * theta;
% del_g_dot = Kw * theta_dot;
% 
% %%Ksi
% a = 1;
% K_n = diag([1 1 1]);
% 
% ksi_1 = -a*del_g;
% ksi_1_dot = -a*del_g_dot;
% 
% K_q_d = diag([0.1 0.1 0.1]);
% ksi_2_simp = -K_q_d*theta_dot + ksi_1; 
%% 3.2.3 Simplified without M eqn.43
% %Gradient 
Kw = diag([1 0.4 1.3]);
del_g = Kw * theta;
del_g_dot = Kw * theta_dot;

%%Ksi
a = 20;
% K_n = diag([1 1 1]);

ksi_1 = -a*del_g;
ksi_1_dot = -a*del_g_dot;

K_q_d = diag([0.1 2 1]);
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
%% Addition of Controllers
% T=tau+Torque_Impedance_at_hip; % SMC from Abizer + VMC at upper link
% T = Torque_Impedance_at_hip; % VMC at upper link
% T= Torque_PD_coupled ;        % Torque generated from coupled momentum using PD controller
% T=tau_coupled_smc ; % Torque generated from coupled momentum using SMC controller
% T=Torque_SMC ;   % Our proposed sliding mode controller for linear and angular momentum
T = Torque_SMC_Linear + T_posture  ; %  Our proposed SMC for linear motion
% T = Torque_SMC_Angular ; %  Our proposed SMC for Angular motion

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
    % fprintf("error is: ");
    % fprintf("%f %f %f\n", error);
    % fprintf("theta is: ");
    % fprintf("%f %f %f\n", theta1, theta2, theta3);
    % % fprintf("s is: ");
    % % fprintf("%f %f %f\n", s);
    % fprintf("torque is: ");
    % fprintf("%f %f %f\n", T_ankle, T_knee, T_hip);
end

% Publish Torque to Joints
%   tau1.Data =T_ankle; % left ankle pitch
%   tau2.Data =0; % left ankle roll
%   tau3.Data =T_knee; % left knee pitch
%   tau4.Data = T_hip ; % left hip pitch
%   tau5.Data =0;  % left hip roll
%   tau6.Data =T_ankle ; % right ankle pitch
%   tau7.Data =0; % right ankle roll
%   tau8.Data =T_knee; % right knee pitch
%   tau9.Data =T_hip; % right hip pitch 
%   tau10.Data = 0 ; % right hip roll
%   send(j1_effort,tau1);
%   send(j2_effort,tau2);
%   send(j3_effort,tau3);
%   send(j4_effort,tau4);
% %   send(j5_effort,tau5);
%   send(j6_effort,tau6);
%   send(j7_effort,tau7);
%   send(j8_effort,tau8);
%   send(j9_effort,tau9);
    tau_msg.Data = [0, 0, T_hip, T_knee, T_ankle, 0, ...
                    0, 0, T_hip, T_knee, T_ankle, 0];
    send(effort_controller, tau_msg)

 
  Theta_COM(end+1)=Angle_COM;
  W_velocity_COM(end+1)=W_COM;
  Position_COM(end+1)=X_COM;
  Linear_velocity_COM(end+1)=X_dot_COM;
  Time(end+1)=t;
end
%  Theta_COM';
%  W_velocity_COM';
%  Theta_COM_calculated';
% W_velocity_COM_calculated';
%  Position_COM';
%  Linear_velocity_COM';
%  Time';
%  COP' ; 
%  S1_dot';
%  S2_dot';
%  S1';
%  S2';
 
% T_SMC_Linear';
% T_SMC_Angular';


 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tau1.Data = 0;
% tau2.Data = 0;
% tau3.Data = 0;
% tau4.Data = 0;
% tau5.Data = 0;
% tau6.Data = 0;
% tau7.Data = 0;
% tau8.Data = 0;
% tau9.Data = 0;
% tau10.Data =0;
% send(j1_effort,tau1);
% send(j2_effort,tau2);
% send(j3_effort,tau3);
% send(j4_effort,tau4);
% send(j5_effort,tau5);
% send(j6_effort,tau6);
% send(j7_effort,tau7);
% send(j8_effort,tau8);
% send(j9_effort,tau9);
% send(j10_effort,tau10);
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

figure;
subplot(2,1,1)
plot(Time,S1_dot)
title('S1 dot ')
subplot(2,1,2)
plot(Time,S2_dot)
title(' S2 dot ')

figure;
subplot(2,1,1)
plot(Time,S1 )
title('S1   ')
subplot(2,1,2)
plot(Time,S2 )
title(' S2   ')

figure;
subplot(2,1,1)
plot(Time, Position_COM )
title('Position COM   ')
subplot(2,1,2)
plot(Time,Linear_velocity_COM )
title(' Linear Velocity COM   ')

figure;
subplot(3,1,1)
plot(Time, T_SMC_Linear(1,:) ,'r' )
title(' Ankle Torque   ')
hold on
plot(Time, T_SMC_Angular(1,:) ,'b' )
hold off
subplot(3,1,2)
plot(Time, T_SMC_Linear(2,:) ,'r' )
title(' Knee Torque   ')
hold on
plot(Time, T_SMC_Angular(2,:) ,'b' )
 hold off
subplot(3,1,3)
plot(Time, T_SMC_Linear(3,:) ,'r' )
title(' Hip  Torque   ')
hold on
plot(Time, T_SMC_Angular(3,:) ,'b' )

figure;
subplot(2,1,1)
plot(Time, Theta_COM_calculated)
title('Theta_COM_calculated   ')
subplot(2,1,2)
plot(Time,W_velocity_COM_calculated )
title(' W_velocity_COM_calculated   ')
 

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