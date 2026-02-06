% build_two_link_one_joint_robot.m
%
% PURPOSE
%   Build a minimal robot model in MATLAB using rigidBodyTree:
%     - 2 links (link1 and end-effector link)
%     - 1 revolute joint
%
%   This serves as a template for students to add additional links and joints.
%
% STUDENT EXTENSION
%   To add more links:
%   1. Define new link length (e.g., L2 = 0.25)
%   2. Create new body and joint
%   3. Set joint axis and fixed transform
%   4. Add to robot with appropriate parent

clear; clc; close all

%% 1) Create the rigid body tree container
robot = rigidBodyTree( ...
    "DataFormat","row", ...     % q is a 1xN row vector [q1 q2 ...]
    "MaxNumBodies",5);          % Preallocate space (increase if adding more links)

%% 2) Define link lengths
L1 = 0.30;  % [m] length of link1

% STUDENT TODO: Add more link lengths here
% L2 = 0.25;  % [m] length of link2
% L3 = 0.20;  % [m] length of link3

%% 3) Add first link: "link1" attached to base
body1 = rigidBody("link1");

j1 = rigidBodyJoint("joint1","revolute");
j1.JointAxis = [0 0 1];         % Rotation axis: [0 0 1] for +Z

setFixedTransform(j1, trvec2tform([0 0 0]));  % Joint at base origin

body1.Joint = j1;
addBody(robot, body1, "base");

%% 4) Add end-effector link
ee = rigidBody("ee_link");

j_ee = rigidBodyJoint("joint_ee","revolute");
j_ee.JointAxis = [0 0 1];

setFixedTransform(j_ee, trvec2tform([L1 0 0]));  % At end of link1

ee.Joint = j_ee;
addBody(robot, ee, "link1");

%% STUDENT TODO: Add additional links here following this pattern:
%{
body2 = rigidBody("link2");
j2 = rigidBodyJoint("joint2","revolute");
j2.JointAxis = [0 0 1];  % Change axis as needed
setFixedTransform(j2, trvec2tform([L2 0 0]));
body2.Joint = j2;
addBody(robot, body2, "link1");  % Parent to previous link

% Then move ee_link to attach to the new link2 instead
%}

%% 5) Visualize the robot
figure("Name","MATLAB robot show");
show(robot);
title("2-link, 1-revolute-joint robot");
view(0,90)
camproj('orthographic')

%% 6) Test forward kinematics with sample configuration
q = [0.5, -0.5];   % [rad] joint angle for joint1

T = getTransform(robot, q, "ee_link", "base");

disp("End effector transform (base -> ee_link):");
disp(T);
disp("End effector position [x y z] (m):");
disp(tform2trvec(T));

%% 7) Visualize at specified configuration
figure("Name","Robot at specified joint configuration q");
show(robot, q, "Frames","on");
title("Robot at specified joint configuration q");
axis equal;
view(0,90)
camproj('orthographic')
