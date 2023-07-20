clear; close all; clc
syms s Q  a c k   z p   R u SO f G  t x theta1 theta1 theta2 theta2 theta3 theta3 l1 l2 l3 m1 m2 m3 lc1 lc2 lc3 'real'
% syms s Q  a c k   z p   R u SO f G  x theta1  theta2  theta3 theta1_dot theta2_dot theta3_dot  l1 l2 l3 m1 m2 m3 lc1 lc2 lc3 'real'
assumeAlso(m1>0 & m2>0 & m3>0 & l1>0 & l2>0 & l3>0 & lc1>0 & lc2>0 & lc3>0)

r = [ ...
    ( (lc1*cos(theta1))*m1 + (l1*cos(theta1)+lc2*cos(theta1+theta2))*m2 + (l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); ...
    -1*((lc1*sin(theta1))*m1 + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2 + (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3) / (m1+m2+m3); ...
    0];

v = simplify(diff(r, t))
% v = [ ...
%     -(m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3);
%     -(m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3);
%                                                                                                                                                                                                                                                                                                                                                                                                                   0];
% v = [...
% -(m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3);
% -(m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3);
% 0]
% 
% w = simplify(cross(r, v)/norm(r)^2)
w = -((((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)*(l1*m2*sin(theta1) + l1*m3*sin(theta1) + lc1*m1*sin(theta1) + lc3*m3*sin(theta1 + theta2 + theta3) + l2*m3*sin(theta1 + theta2) + lc2*m2*sin(theta1 + theta2)))/(m1 + m2 + m3)^2 + ((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)*(l1*m2*cos(theta1) + l1*m3*cos(theta1) + lc1*m1*cos(theta1) + lc3*m3*cos(theta1 + theta2 + theta3) + l2*m3*cos(theta1 + theta2) + lc2*m2*cos(theta1 + theta2)))/(m1 + m2 + m3)^2)*(m1 + m2 + m3)^2)/(abs(l1*m2*cos(theta1) + l1*m3*cos(theta1) + lc1*m1*cos(theta1) + lc3*m3*cos(theta1 + theta2 + theta3) + l2*m3*cos(theta1 + theta2) + lc2*m2*cos(theta1 + theta2))^2 + abs(l1*m2*sin(theta1) + l1*m3*sin(theta1) + lc1*m1*sin(theta1) + lc3*m3*sin(theta1 + theta2 + theta3) + l2*m3*sin(theta1 + theta2) + lc2*m2*sin(theta1 + theta2))^2)


% th = int(w, t)
