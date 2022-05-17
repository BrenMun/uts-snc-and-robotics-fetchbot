%% Assignment 2
clf; clc; clear all;
% profile on; 

% Variables
taskCompleted = false; %Boolean for task completed 
workspace = [-0.2 1.7 -1.2 1.2 0.2 1.5]; % workspace for matlab simulation
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

% calling percetion 
%system('cd /home/ian/git/uts-snc-and-robotics-fetchbot/fetchbot_src/build && rosrun fetchbot perception')
%pause(5)
% Grab object location from camera in ROS
headCamera.perceptionFcn(headCamera.subPoint.LatestMessage, headCamera.subCloud.LatestMessage);
object = headCamera.targetPoint;
% add object to matlab simulation
simulation.addObject(1,object);


%% Teach for gui control
simulation.teaching; 
%% Get arms current position
simulation.getPos()
%% Test movement
%qObject = [0    0.8219   -0.0780   -0.8516   -1.2403    1.0248   -1.4547    2.6003];
%qObject = [0    0.9346    0.0698    1.5080   -0.3339    0.1255   -2.0972    1.2579];
qObject = [0    1.4806   -0.0398    0.0000   -0.5591   -0.0001    1.5680    0.0012];
simulation.MoveArm(qObject);
%% Move to bin
qObject =[0.0640    1.3200   -0.1495   -0.1256   -0.8292   -0.1258    0.8263    0.0012];
simulation.MoveArm(qObject);
%% check current pos for collisions with table
simulation.checkCollisions(simulation.robotFetch);
%% simulation reset
qObject = simulation.robotFetch.qHome;
simulation.MoveArm(qObject);
%%
simulation.getFkine();
%% Recycle Object

while (taskCompleted == false && eStop == false)
    simulation.Recycle();

    if (simulation.robotFetch.taskcompleted == true)
        taskCompleted = true;
        disp("Object Recycled");
    end
end

% profile viewer
        