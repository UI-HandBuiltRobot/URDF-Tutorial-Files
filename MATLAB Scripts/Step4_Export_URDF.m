%% Step 4: Export to URDF

% Pick a folder to export the URDF file
[exportname exportDir] = uiputfile("My_URDF.urdf")

% Export the rigidBodyTree to URDF
exportrobot(robot, ...
    "OutputFileName", [exportDir exportname]);

disp("Wrote URDF to:");
disp([exportDir exportname]);
