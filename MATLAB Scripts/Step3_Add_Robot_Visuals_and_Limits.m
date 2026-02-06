%% Step 3: Add Robot Visuals and Joint Limits

%% Get handles to bodies in the robot
b1 = getBody(robot, "link1");
ee = getBody(robot, "ee_link");

% STUDENT TODO: Get additional bodies as you add them
% b2 = getBody(robot, "link2");
% b3 = getBody(robot, "link3");

%% Get joint handles
j1 = b1.Joint;
j_ee = ee.Joint;

% STUDENT TODO: Get additional joints as you add them
% j2 = b2.Joint;
% j3 = b3.Joint;

%% Set joint limits
j1.PositionLimits = [-pi, pi];      % [rad] joint1 limits
j_ee.PositionLimits = [-pi, pi];    % [rad] ee joint limits

% STUDENT TODO: Set limits for additional joints
% j2.PositionLimits = [-pi, pi];
% j3.PositionLimits = [-pi, pi];

%% Set home positions
j1.HomePosition = 0.0;
j_ee.HomePosition = 0.0;

% STUDENT TODO: Set home positions for additional joints
% j2.HomePosition = 0.0;
% j3.HomePosition = 0.0;

%% Add visual geometry
linkThickness = 0.04;  % [m] thickness of link boxes
eeSize = 0.05;         % [m] size of end-effector cube

% Add visuals centered along each link
addVisual(b1, "Box", [L1, linkThickness, linkThickness], trvec2tform([L1/2, 0, 0]));
addVisual(ee, "Box", [eeSize, eeSize, eeSize], trvec2tform([0, 0, 0]));

% STUDENT TODO: Add visuals for additional links
% addVisual(b2, "Box", [L2, linkThickness, linkThickness], trvec2tform([L2/2, 0, 0]));
% addVisual(b3, "Box", [L3, linkThickness, linkThickness], trvec2tform([L3/2, 0, 0]));

%% Visualize at home configuration
figure("Name","Robot at zero configuration");
show(robot, homeConfiguration(robot), "Frames","on",'Visuals','on');
title("Robot at home (zero) configuration");
axis equal;
view(0,90)
camproj('orthographic')
lighting gouraud

%% Visualize at specified configuration q
figure("Name","Robot at specified joint configuration q");
show(robot, q, "Frames","on",Visuals="on");
title("Robot at specified joint configuration q");
axis equal;
view(0,90)
camproj('orthographic')
lighting gouraud
