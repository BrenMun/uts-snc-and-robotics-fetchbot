%% Assignment 2
clf; clc; clear all;

% Variables
taskCompleted = false; %Boolean for task completed 
workspace = [-0.2 1.7 -1 1 0.2 1.5]; % workspace for matlab simulation
centerpnt = [1,0,0.25];  % Centre point for table proxy in matlab
binPoint = [-0.2,0.5,0]; % bin centre point 

% estop functionality
eStop = false; 

% Add starting Poses for Fetch Robot
fetchBase = transl(0,0,0.7) *trotz(pi);

% Creates object conenct to ROS node for head camera information
headCamera = RobotTemplateClass(getenv("ROS_IP"));

% Generate the Simulation
simulation = Simulation(fetchBase, workspace, centerpnt, binPoint, headCamera);

% Grab object location from camera in ROS
headCamera.perceptionFcn(headCamera.subPoint.LatestMessage, headCamera.subCloud.LatestMessage);
object = headCamera.targetPoint;

% add object to matlab simulation
simulation.addObject(1,object)

%% Recycle Object
while (taskCompleted == false && eStop == false)
    simulation.Recycle(simulation.robotFetch);
    if (simulation.robotFetch.taskcompleted == true)
        taskCompleted = true;
        disp("Object Recycled");
    end
end

rosshutdown