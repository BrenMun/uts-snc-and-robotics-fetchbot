clear; clc; close all; 

%% SETUP
robot = RobotTemplateClass(getenv("ROS_IP"));

%% CONTROL LOOP
tic
currentTime = 0;
while(currentTime < 10)
    %% 1: SENSE, 2: PROCESS and 3: CONTROL
    % Get latest data from ROS subscribers
    % Run perception and control algorithms, which use received data and 
    % control parameters to produce some output.
    % Package and send control outputs as ROS messages
    robot.perceptionFcn(robot.MySub.LatestMessage);
    robot.controlFcn;

    %% 4: VISUALIZE
    % (Optional) Visualize data as the algorithm is running
    currentTime = toc;
    plot(robot.MyAxes,currentTime,robot.ControlOutputs,'bo','MarkerSize',5)
    drawnow

    % (Optional) Pause execution to add delay between iterations
    pause(0.1)
end