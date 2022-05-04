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
    robot.perceptionFcn(robot.subPoint.LatestMessage, robot.subCloud.LatestMessage); 
%     robot.controlFcn 
    pause(0.1)
end

% shut down node after loop
rosshutdown