clc; clear; close all

robot = importrobot("urdf/huron_description_3.urdf", "DataFormat", "row");
robot.Gravity = [0 0 -9.8];
% show(robot, "Visuals","off","Collisions","on")
show(robot)