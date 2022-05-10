%% Assignment 2
clf; clc; clear all;
% profile on; 

% Variables
taskCompleted = false; %Boolean for task completed 
workspace = [-0.2 1.7 -1 1 0.2 1.5]; % workspace for matlab simulation
centerpnt = [1,0,0.25];  % Centre point for table proxy in matlab
binPoint = [-0.2,0.5,0]; % bin centre point 

% estop functionality
eStop = false; 

% Add starting Poses for Fetch Robot
fetchBase = transl(0,0,0.7) *trotz(pi);

% Creates object conenct to ROS node for head camera inform                              ation
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
simulation.teaching; 
%% Get arms current position
simulation.getPos()
%% Test movement
simulation.TestMovement(simulation.robotFetch);
%% add way point
waypoint = [0    0.8219   -0.8139   -3.3648   -1.1505   -1.7399   -2.0217 2.6003];
simulation.AddWayPoint(simulation.robotFetch, waypoint);
%% Testing Collision detection - use way point interpolation
waypoint = [0    0.9346    0.0972    0.1257   -2.0000   -0.1258   -0.5700    0.0012];
simulation.addWayPoint(simulation.robotFetch, waypoint);
pause(2);
waypoint = [0    0.9346    0.0698    1.2567   -0.9643    0.2512   -0.8754    0.001];
simulation.addWayPoint(simulation.robotFetch, waypoint);
pause(2);
waypoint = [0    0.9346    0.0698    1.5080   -0.3339    0.1255   -2.0972    1.2579];
simulation.addWayPoint(simulation.robotFetch, waypoint);

%%
simulation.moveThroughWaypoints(); 

%% check current pos for collisions with table
simulation.checkCollisions(simulation.robotFetch);

%% simulation reset
simulation.resetRobot(simulation.robotFetch);

%% testing grip
simulation.grip(0.0);

%% Recycle Object

while (taskCompleted == false && eStop == false)
    simulation.Recycle(simulation.robotFetch);

    if (simulation.robotFetch.taskcompleted == true)
        taskCompleted = true;
        disp("Object Recycled");
    end
end

% profile viewer
        