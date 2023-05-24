clc; clear; close all

robot = importrobot("urdf/huron_description.urdf", "DataFormat", "row");
robot.Gravity = [0 0 -9.8];
% show(robot, "Visuals","off","Collisions","on")
show(robot, "Visuals", "off")