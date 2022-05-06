%% Ian Chivers 13283837 Assignment 1
clf; clc; clear all;
% profile on; 

% Variables
taskCompleted = false; %Boolean for task completed 
workspace = [-1 1 -1 1 0 1.5]; % workspace for matlab simulation
centerpnt = [0,0.3,0];  % Centre point for table proxy in matlab
binPoint = [-0.2,0.5,0]; % bin centre point 

% Add starting Poses for Fetch Robot
fetchBase = transl(0,-0.8,0.5) *trotz(-pi/2);

% Creates object conenct to ROS node for head camera information
headCamera = RobotTemplateClass(getenv("ROS_IP"));

% Generate the Simulation
simulation = Simulation(fetchBase, workspace, centerpnt, binPoint, headCamera);

% Grab object location from camera in ROS
headCamera.perceptionFcn(headCamera.subPoint.LatestMessage, headCamera.subCloud.LatestMessage);
object = headCamera.targetPoint;
% add object to matlab simulation
simulation.addObject(1,object)

%% Max Reach (To Do)
%simulation.robotFetch.CalculateMaxReach(fetchBase)
%% (Add more layers)
%simulation.robotFetch.FetchRobotReachVolume() 
%% Teach for gui control
%simulation.teaching; 
%% Recycle Object

while taskCompleted == false
    simulation.Recycle(simulation.robotFetch);

    if (simulation.robotFetch.taskcompleted == true)
        taskCompleted = true;
        disp("Object Recycled")
    end
end

% profile viewer
        