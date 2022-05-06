%% Ian Chivers 13283837 Assignment 1
clf 
clc 
clear all

% profile on; 

% Variables
brickNum = 1;
tableTolerance = 0.75;
taskCompleted = false; 
workspace = [-2 2 -2 2 -1.2 2.5];
brickX=[];
brickY=[];
brickZ=[];
brickRot=[];

% Add starting Poses for Robots
ur3Base =  transl(0,0.5,0) * trotz(0);  
ur5Base = (transl(0,-0.5,0) * trotx(pi/2) * troty(pi/2) * trotz(0));
fetchBase = transl(-0.8,0.5,0.15) *trotz(pi);

% Add Bricks
random = 0; % 1 for random or 0 for manual values.

switch random
    case 0
%         brickX=[0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6];
%         brickY=[-0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8];
%         brickZ=[0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04];
        brickX=[0.1, 0.3, 0.2, 0.1];
        brickY=[0.5, 0.4, 0.6, 0.8];
        brickZ=[0.04, 0.04, 0.04, 0.04];
        brickRot= zeros(1,5);  %fill with zeros, fix class
        
    case 1
        for i = 1:1:brickNum
%             a = -1.75/2 + tableTolerance;
%             b = 1.75/2 - tableTolerance;
%             rx = (b-a).*rand(1000,1) + a;
%             rx_range = [min(rx) max(rx)];
%            
%             q = -3.25/2 + tableTolerance;
%             u = 3.25/2 - tableTolerance;
%             ry = (u-q).*rand(1000,1) + q;
%             ry_range = [min(ry) max(ry)];
%         
%             brickX(end+1) = rx(1); 
%             brickY(end+1) = ry(1);
%             brickZ(end+1) = 0.04;
        brickX(end+1) = [0.6];
        brickY(end+1) = [-1.0 + i*0.2];
        brickZ(end+1) = 0.04;

        end
    otherwise 
end

% Generate the Simulation
simulation = Simulation(ur3Base,ur5Base,fetchBase,brickNum,brickX,brickY,brickZ);

%%
simulation.robotFetch.CalculateMaxReach(fetchBase)
%%
simulation.robotFetch.FetchRobotReachVolume() 

% Find The UR3 Robots Reach
% simulation.robotUR3.CalculateMaxReach(ur3Base);
% Find The UR5 Robots Reach
% simulation.robotUR5.CalculateMaxReach(ur5Base);
% Find The UR3 Robots Volume
% simulation.robotUR3.UR3ReachVolume()
% Find The UR5 Robots Volume
% simulation.robotUR5.UR5ReachVolume()

%% Teach
% robotFetch = FetchRobot(fetchBase);
% robotFetch.teaching();
simulation.teaching; 

%% Build Wall

simulation.DetermineWallLocation()

while taskCompleted == false
    % simulation.BuildBrickWall(simulation.robotUR5);
    simulation.BuildBrickWall(simulation.robotFetch);
    if ((simulation.robotFetch.taskcompleted == true)) % && (simulation.robotUR5.taskcompleted == true))
        taskCompleted = true;
    end
end

% profile viewer
        