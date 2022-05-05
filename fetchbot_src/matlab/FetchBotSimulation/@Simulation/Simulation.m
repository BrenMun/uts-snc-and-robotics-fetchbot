classdef Simulation < handle % Passes by reference
    %SIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Handle
        simulation

        %Objects
        robotUR3;
        robotUR5;
        robotFetch;
        environment;
        bricks = Brick.empty;
        brickNum;

        % Variables
        tableTolerance = 0.5;
        workspace = [-4 4 -4 4 -1.2 2.5];
        brickWallPoses= zeros(9,3); 
        brickWallIndex = 1; 
        ur3Base; 
        ur5Base;
        brickX; 
        brickY; 
        brickZ;
        trajSteps = 50; 
        wallX = [];
        wallY = [];
        wallZ = [];

    end

    properties(Constant)

        % Cases for state machine
        LocateNextBrick = 0; 
        PickUpBrick = 1; 
        PlaceBrick = 2; 
        ReturnHome = 3; 
        Uknown = 4; 

        % Brick states for State Machine
        state = 0; 
        unmoved = 0;
        targeted = 1; 
        moving = 3; 
        moved = 4; 
        unknown = 5;

    end

    
    methods
        function obj = Simulation(ur3Base, ur5Base,fetchBase, brickNum, brickX, brickY, brickZ)
            %SIMULATION Construct an instance of this class
            %   Detailed explanation goes here
            obj.brickX = brickX; obj.brickY = brickY; obj.brickZ = brickZ;
            obj.ur3Base = ur3Base; obj.ur5Base = ur5Base; obj.brickNum = brickNum;
            % Simulate Robots
            % Adding UR3 robot
            %obj.robotUR3 = UR3(ur3Base);

            % Adding UR5 robot
            obj.robotUR5 = LinearUR5e(ur5Base);

            % Adding robotFetch
            obj.robotFetch = FetchRobot(fetchBase);
                  
            % Add Table, Cones, Walls
            obj.environment = EnvironmentSetUp(obj.workspace, 2.25);

            % Adding bricks
            for i = 1:1:brickNum
                obj.bricks(i) = Brick(obj.brickX(i),obj.brickY(i),obj.brickZ(i),0); % must fix up adding poses
                %obj.bricks(i).brickWallIndex = i; 
            end
        end
        
        function teaching(obj)
            obj.robotFetch.teaching();
        end



        function DetermineWallLocation(obj) % hard Coded

            brickWidth  = 0.275;
            brickLength = 0.1335;
            brickHeight = 0.0735; 

            X1 = 0 - brickWidth;
            X2 = X1 + brickWidth;
            X3 = X2 + brickWidth;
            Y = 0*brickLength; % 0
            Z1 = 0.04;
            Z2 = 0.04 + brickHeight;
            Z3 = 0.04 + brickHeight*2;
    
            Poses1 = [X1, Y, Z1];
            Poses2 = [X2, Y, Z1];
            Poses3 = [X3, Y, Z1];
            Poses4 = [X1, Y, Z2];
            Poses5 = [X2, Y, Z2];
            Poses6 = [X3, Y, Z2];
            Poses7 = [X1, Y, Z3];
            Poses8 = [X2, Y, Z3];
            Poses9 = [X3, Y, Z3];
    
            brickWallPoses = [Poses1; Poses2; Poses3;
                              Poses4; Poses5; Poses6;
                              Poses7; Poses8; Poses9];
            obj.brickWallPoses = brickWallPoses; 
               
        end


        function BuildBrickWall(obj, robot)
            % State Machine toobj.trajSteps operate robots

            switch robot.currentState

                case obj.LocateNextBrick
                    for i = 1:1:obj.brickNum
                        if  obj.bricks(i).state == obj.unmoved % check bricks state
                            % Find the closest brick, sort through for loop
                            smallestDist = 4.0; % setting high for first run
                            closestBrick = 0; 
                            for j = 1:1:obj.brickNum
                                if  obj.bricks(j).state == obj.unmoved
                                    qEndEff = robot.model.getpos;
                                    qBrick = obj.bricks(j).brickPose;
                                    L = robot.model.fkine(qEndEff);
                                    R = qBrick;
                                   
                                    %Find Distance
                                    distance = sqrt(sum((L(1:3,4)-R(1:3,4)).^2));
                                    if distance <= smallestDist
                                        smallestDist = distance;
                                        closestBrick = j; 
                                    end
                                end
                            end
                            
                            obj.bricks(closestBrick).state = obj.targeted; % set brick state to trageted
                            robot.targetedBrickID = closestBrick; % Set brick id as brick num
                            robot.steps = 1; % resets steps
                            robot.currentState = obj.PickUpBrick; % update robots state
                            robot.previousState = obj.LocateNextBrick; % update robots previous state
                            robot.taskcompleted = false; % update robots task completetion state
                            break;
                        else
                            % End robot operation
                            robot.taskcompleted = true;
                        end
                    end
            
                case obj.PickUpBrick
                    if ((robot.currentState == obj.PickUpBrick) && (robot.previousState ==obj.LocateNextBrick))
                        qCurrent = robot.model.getpos; 
                        poseBrick = obj.bricks(robot.targetedBrickID).brickPose;
                                                
                        qBrick= robot.model.ikcon(poseBrick, qCurrent);
                        robot.armTraj = jtraj(qCurrent, qBrick, obj.trajSteps);
                        robot.previousState = obj.PickUpBrick;
                        robot.steps = 1;
                    else
                        if robot.steps <= obj.trajSteps
                            %for i = 1:1:obj.trajSteps
                                result(robot.steps) = IsCollision(robot,robot.armTraj(robot.steps,:),faces,vertex,faceNormals,false);
                                robot.model.animate(robot.armTraj(robot.steps,:));
                                robot.steps = robot.steps + 1;
                           % end
                        else 
                            obj.bricks(robot.targetedBrickID).state = obj.moving;
                            robot.currentState = obj.PlaceBrick;
                            % obj.brickWallIndex = obj.brickWallIndex + 1;
                            robot.steps = 1;
                        end
                        
                    end

                    case obj.PlaceBrick
                    if ((robot.previousState == obj.PickUpBrick) && (obj.brickWallIndex < 10))
                        qCurrent = robot.model.getpos;
                        qWallPose = transl(obj.brickWallPoses(obj.brickWallIndex,:));   %transl(obj.wallX(obj.brickWallIndex),obj.wallY(obj.brickWallIndex),obj.wallZ(obj.brickWallIndex));
%                         qWallPoses = obj.brickWallPoses(obj.brickWallIndex,:);
%                         qWallPose = [qWallPoses(1)-robot.model.base(1,4), qWallPoses(2)-robot.model.base(2,4), qWallPoses(3)-robot.model.base(3,4)]
%                         qWallPose = transl(qWallPose)
                        %qWallPose = transl(obj.brickWallPoses(obj.bricks(robot.targetedBrickID).brickWallIndex));

                         if obj.bricks(robot.targetedBrickID).state == obj.moving
                             obj.brickWallIndex = obj.brickWallIndex + 1;
                         end
                        
                        qWall = robot.model.ikcon(qWallPose, qCurrent);
                        robot.model.fkine(qWall);
                        robot.armTraj = jtraj(qCurrent, qWall, obj.trajSteps);
                        robot.previousState = obj.PlaceBrick;
                        robot.steps = 1;
                    else
                       if robot.steps <= obj.trajSteps 
                           robot.model.animate(robot.armTraj(robot.steps,:));
                           robot.steps = robot.steps + 1;
                           % Update the brick position as robot moves
                           % through trajectories
                           updatedBrickPose = robot.model.fkine(robot.model.getpos);
                           obj.bricks(robot.targetedBrickID).updateBrickPose(updatedBrickPose); % as robot trajectory changes, update bricks is called
                       else
                           obj.bricks(robot.targetedBrickID).state = obj.moved;
                           robot.currentState = obj.LocateNextBrick;
                           robot.steps = 1;
                       end
                    end
 
            end
        end
%%

%% IsCollision
% This is based upon Lab 5 exercises
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
%     q = qMatrix(qIndex,:);
    
    % Get the transform of every joint (i.e. start and end of every link)  
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% GetLinkPoses
function [ transforms ] = GetLinkPoses( q, robot)
%q - robot joint angles
%robot -  seriallink robot model
%transforms - list of transforms

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);

    transforms(:,:,i + 1) = current_transform;
end
end


%%        
    end
end

