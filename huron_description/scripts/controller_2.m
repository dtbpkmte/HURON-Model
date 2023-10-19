%syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 I1 I2 I3 theta1 theta2 theta3 theta1_dot theta2_dot theta3_dot g

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

 
tOld = 0;
% Main Loop
count = 1;
while(t <= 10)
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

%% EOM of 3 DOF model from (Achievement of chaotic paper).
% % Physcial Parameters Values
% m1=5.9117; m2=4.2554; m3=10.19329; lc1=0.15149; lc2=0.24517 ; lc3=0.1585; l1=0.3715; l2=0.49478; l3=0.32662; g=9.81; I1=0.0222 ; I2=0.01009 ;I3=0.0219 ; % mass in kg, length in meter
% % Mass Matrix
% M11=-lc1^2 *m1-(l1^2+lc2^2+2*l1*lc2*cos(theta2))*m2-(l1^2+l2^2+lc3^2+2*l1*l2*cos(theta2)+2*l2*lc3*cos(theta3)+2*l1*lc3*cos(theta2+theta3))*m3+I1+I2+I3;
% M12=-(lc2^2+l1*lc2*cos(theta2))*m2-(l2^2+lc3^2+l1*l2*cos(theta2)+2*l2*lc3*cos(theta3)+l1*lc3*cos(theta2+theta3))*m3+I2+I3;
% M21=-(lc2^2+l1*lc2*cos(theta2))*m2-(l2^2+lc3^2+l1*l2*cos(theta2)+2*l2*lc3*cos(theta3)+l1*lc3*cos(theta2+theta3))*m3+I2+I3;
% M13=-(lc3^2+l1*lc3*cos(theta2+theta3)+l2*lc3*cos(theta3))*m3+I3;
% M31=-(lc3^2+l1*lc3*cos(theta2+theta3)+l2*lc3*cos(theta3))*m3+I3;
% M22=-lc2^2*m2-(l2^2+lc3^2+2*l2*lc3*cos(theta3))*m3+I2+I3;
% M23=-(lc3^2+l2*lc3*cos(theta3))*m3+I3;
% M32=-(lc3^2+l2*lc3*cos(theta3))*m3+I3;
% M33=-lc3^2*m3+I3;
% M=[M11 M12 M13 ; M21 M22 M23 ; M31 M32 M33];
% % Coriolous Force
% C1=(m2*l1*lc2*sin(theta2)+m3*l1*l2*sin(theta2)+m3*l1*lc3*sin(theta2+theta3))*(theta2_dot)^2 + (m3*l2*lc3*sin(theta3)+m3*l1*lc3*sin(theta2+theta3))*(theta3_dot)^2 + 2*(m2*l1*lc2*sin(theta2)+m3*l1*l2*sin(theta2)+m3*l1*lc3*sin(theta2+theta3))*theta1*theta2 + 2*(m3*l2*lc3*sin(theta3)+m3*l1*lc3*sin(theta2+theta3))*theta2*theta3 + 2*(m3*l2*lc3*sin(theta3)+m3*l1*lc3*sin(theta2+theta3))*theta1*theta3 ;
% C2=-(m2*l1*lc2*sin(theta2)+m3*l1*l2*sin(theta2)+m3*l1*lc3*sin(theta2+theta3))*(theta1_dot)^2 + m3*l2*lc3*sin(theta3)*(theta3_dot)^2 + 2*m3*l2*lc3*sin(theta3)*theta2*theta3 + 2*m3*l2*lc3*sin(theta3)*theta1*theta3;
% C3=-(m3*l2*lc3*sin(theta3)+m3*l1*lc3*sin(theta2+theta3))*(theta1_dot)^2 - m3*l2*lc3*sin(theta3)*(theta2_dot)^2 - 2*m3*l2*lc3*sin(theta3)*theta1*theta2 ;
% C=[C1 ; C2 ;C3]; % Coriolous Vector
% % Gravitaional Vector
% g1= -lc1*sin(theta1)*m1*g-(l1*sin(theta1)+lc2*sin(theta1+theta2))*m2*g - (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3*g;
% g2= -lc2*sin(theta1+theta2)*m2*g - (l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m2*g;
% g3=-lc3*sin(theta1+theta2+theta3)*m3*g;
% G=[g1 ; g2 ; g3]; % gravitaional vector
% N=C+G;
% %% end of 3 DOF model from ( Achievement of chaotic paper )................
% 
% %% Center of Mass COM calculation.
% X_COM=( (lc1*sin(theta1))*m1 + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2 + (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); % Center of Mass position in x_direction
% X_dot_COM=(  m1*(theta1_dot*lc1*cos(theta1)) + m2*(theta1_dot*l1*cos(theta1) +(theta1_dot+theta2_dot)*lc2*cos(theta1+theta2)) + m3*(theta1_dot*l1*cos(theta1)+(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)+(theta1_dot+theta2_dot+theta3_dot)*lc3*cos(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in x_direction
% 
% J_X_COM=[(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1))/(m1 + m2 + m3), (m3*(l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3)) + lc2*m2*cos(theta1 + theta2))/(m1 + m2 + m3), (lc3*m3*cos(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of X-COM Jx
% 
% J_X_COM_dot=[(-m1*theta1_dot*lc1*sin(theta1) + m2*(-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) )  + m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2)-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3) ,( m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2) -lc3*(theta1_dot+theta2_dot+theta3_dot)*sin(theta1+theta2+theta3) ) -lc2*(theta1_dot+theta2_dot)*m2*sin(theta1+theta2) )/(m1+m2+m3) ,(-lc3*(theta1_dot+theta2_dot+theta3_dot)*m3*sin(theta1+theta2+theta3) )/(m1+m2+m3)]; % Time Derivative of Jacobian Matrix
% 
% Pseudo_J_X_COM=pinv(J_X_COM); % Pseudo Inverse of J_X_COM
% 
% Z_COM=( (lc1*cos(theta1))*m1 + (l1*cos(theta1)+lc2*cos(theta1+theta2))*m2 + (l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); % Center of Mass position in z_direction
% Z_dot_COM=(  -m1*(theta1_dot*lc1*sin(theta1)) + m2*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) ) + m3*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*l2*sin(theta1+theta2) - (theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in z_direction
% 
% J_Z_COM=[(m3*(-l2*sin(theta1 + theta2) - l1*sin(theta1) - lc3*sin(theta1 + theta2 + theta3)) + m2*(-lc2*sin(theta1 + theta2) - l1*sin(theta1)) - lc1*m1*sin(theta1))/(m1 + m2 + m3), (m3*(-l2*sin(theta1 + theta2) - lc3*sin(theta1 + theta2 + theta3)) - lc2*m2*sin(theta1 + theta2))/(m1 + m2 + m3), (-lc3*m3*sin(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of Z-COM Jz
% 
% J_COM=[J_X_COM ; J_Z_COM];
% 
% %% end of COM Calculation
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
%% Add the controller here
%% PID control
% error = [theta1 - theta1_d; theta2 - theta2_d; theta3 - theta3_d];
% 
% error_dot = [theta1_dot - theta1_dot_d; theta2_dot - theta2_dot_d; theta3_dot - theta3_dot_d];
% Kp = diag([1500,1500,1500]);
% Kd = diag([10,10,10]);
% tau = -Kp*error - Kd*error_dot;
% T_ankle = tau(1);
% T_knee = tau(2);
% T_hip = tau(3);


%% Computed torque control
% error = [theta1 - theta1_d; theta2 - theta2_d; theta3 - theta3_d; theta1_dot - theta1_dot_d; theta2_dot - theta2_dot_d; theta3_dot - theta3_dot_d];
% error_des = [0;0;0;0;0;0];
% vd = 0;
% A = [zeros(3) eye(3); zeros(3) zeros(3)];
% B = [zeros(3); eye(3)];
% K = place(A,B,[-0.5 -0.4 -0.3 -0.2 -0.1 -0.15]);
% vc = -K*(error-error_des) + vd;
% 
% tau = M*vc + C + G;
% if tau(1) > 25
%     tau(1) = 25;
% end
% if tau(1) < -25
%     tau(1) = -25;
% end
% T_ankle = tau(1);
% T_knee = tau(2);
% T_hip = tau(3);


%% Sliding mode control based on Peng Ji's paper.
% u1 = 1; u2 = 3; m = 3 ;p = 1; k = 1; l = 1; K1 = [2 2 2];
% K2 = [2 2 2]; phi = 0.75;
% %del = 0.5; alpha = 4; beta = 1; 
% error = -1*[theta1 - theta1_d; theta2 - theta2_d; theta3 - theta3_d];
% 
% error_dot = -1*[theta1_dot - theta1_dot_d; theta2_dot - theta2_dot_d; theta3_dot - theta3_dot_d];
% theta_dddot = [theta1_dddot; theta2_dddot; theta3_dddot]; 
% theta_dot = [theta1_dot; theta2_dot; theta3_dot];
% 
% 
% %s = error_dot + u1*error.^(m/p) + u2*error.^(k/l);
% %s = error_dot + lambda*error;
% if norm(s) >= phi
%     sat = sign(s);
% else
%     sat = s / phi;
% end
% tau = M*(theta_dddot + u1*(m/p)*diag(error.^((m/p)-1))*error_dot + u2*(k/l)*diag(error.^((k/l)-1))*error_dot...
% +diag(K1)*s + diag(K2)*sat) + C + G;
% T_ankle = tau(1);
% T_knee = tau(2);
% T_hip = tau(3);



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
if tau(1) >= 30
    tau(1) = 30;
end
if tau(1) <= -11.76
    tau(1) = -11.76;
end
T_ankle = tau(1);
T_knee = tau(2);
T_hip = tau(3);

%% end of controller

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

 
end
 
 

 
 

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
 