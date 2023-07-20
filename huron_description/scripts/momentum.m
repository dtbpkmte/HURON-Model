clear; close all; clc
% syms s Q  a c k   z p   R u SO f G  t x theta1 theta1 theta2 theta2 theta3 theta3 l1 l2 l3 m1 m2 m3 lc1 lc2 lc3 'real'
% syms s Q  a c k   z p   R u SO f G  x theta1  theta2  theta3 theta1_dot theta2_dot theta3_dot  l1 l2 l3 m1 m2 m3 lc1 lc2 lc3 'real'
syms g I1 I2 I3 s Q  a c k   z p   R u SO f G  x q1  q2  q3 q1_dot q_dot2 q_dot3  l1 l2 l3 m1 m2 m3 lc1 lc2 lc3 'real'
assumeAlso(m1>0 & m2>0 & m3>0 & l1>0 & l2>0 & l3>0 & lc1>0 & lc2>0 & lc3>0)

r1 = lc1;
r2 = lc2;
r3 = lc3;

T1= [ cos(q1)  -sin(q1)*cos(0)  sin(q1)*sin(0)  l1*cos(q1) ;  sin(q1) cos(q1)*cos(0) -cos(q1)*sin(0) l1*sin(q1) ; 0 sin(0) cos(0) 0 ; 0 0 0 1];
T2= [ cos(q2)  -sin(q2)*cos(0)  sin(q2)*sin(0)  l2*cos(q2) ;  sin(q2) cos(q2)*cos(0) -cos(q2)*sin(0) l2*sin(q2) ; 0 sin(0) cos(0) 0 ; 0 0 0 1];
T3= [ cos(q3)  -sin(q3)*cos(0)  sin(q3)*sin(0)  l3*cos(q3) ;  sin(q3) cos(q3)*cos(0) -cos(q3)*sin(0) l3*sin(q3) ; 0 sin(0) cos(0) 0 ; 0 0 0 1];

T = simplify(T1*T2*T3)

G = [- g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*l1*m2*sin(q1) - g*l1*m3*sin(q1) - g*m1*r1*sin(q1) - g*m3*r3*sin(q1 + q2 + q3);
    - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*m3*r3*sin(q1 + q2 + q3);
    -g*m3*r3*sin(q1 + q2 + q3)];

Phi1 = eye(6);

%% Inputs
% Joint-space dynamics descriptors
M = [I1 + I2 + I3 + l1^2*m2 + l1^2*m3 + l2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + 2*l1*m3*r3*cos(q2 + q3) + 2*l1*l2*m3*cos(q2) + 2*l1*m2*r2*cos(q2) + 2*l2*m3*r3*cos(q3), m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3);
m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, m3*l2^2 + 2*m3*cos(q3)*l2*r3 + m2*r2^2 + m3*r3^2 + I2 + I3, m3*r3^2 + l2*m3*cos(q3)*r3 + I3;
I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3), m3*r3^2 + l2*m3*cos(q3)*r3 + I3, m3*r3^2 + I3];

C = [- l1*m3*q_dot2^2*r3*sin(q2 + q3) - l1*m3*q_dot3^2*r3*sin(q2 + q3) - l1*l2*m3*q_dot2^2*sin(q2) - l1*m2*q_dot2^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l1*m3*q1_dot*q_dot2*r3*sin(q2 + q3) - 2*l1*m3*q1_dot*q_dot3*r3*sin(q2 + q3) - 2*l1*m3*q_dot2*q_dot3*r3*sin(q2 + q3) - 2*l1*l2*m3*q1_dot*q_dot2*sin(q2) - 2*l1*m2*q1_dot*q_dot2*r2*sin(q2) - 2*l2*m3*q1_dot*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    l1*m3*q1_dot^2*r3*sin(q2 + q3) + l1*l2*m3*q1_dot^2*sin(q2) + l1*m2*q1_dot^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l2*m3*q1_dot*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    l1*m3*q1_dot^2*r3*sin(q2 + q3) + l2*m3*q1_dot^2*r3*sin(q3) + l2*m3*q_dot2^2*r3*sin(q3) + 2*l2*m3*q1_dot*q_dot2*r3*sin(q3)];

U1 = [eye(6) zeros(6,3)];
% Floating-base kinematics
R_0_1 = T(1:3, 1:3)
Psi1 = inv(Phi1);

%% Computations
M11 = U1*M*U1';
I_C_1 = Psi1'*M11*Psi1
Mass = I_C_1(6,6)
