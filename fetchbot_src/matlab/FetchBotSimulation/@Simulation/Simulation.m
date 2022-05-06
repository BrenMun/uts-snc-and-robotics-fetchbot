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
        workspace;
        binPoint = []; 
        objectLocation;
        headCamera;

        % Variables
        tableTolerance = 0.5;
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

        objectCarteason;

    end

    properties(Constant)


        % Move to bin states
        LocateObject = 0; 
        MoveToObject = 1;
        MoveToBin = 2; 

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
        function obj = Simulation(fetchBase, workspace, centerpnt,binPoint, headCamera)
            %SIMULATION Construct an instance of this class
            %   Detailed explanation goes here
            obj.workspace = workspace; 
            obj.binPoint = binPoint; 
            obj.headCamera = headCamera;
            %obj.ur3Base = ur3Base; obj.ur5Base = ur5Base;
            % Simulate Robots
            % Adding UR3 robot
            %obj.robotUR3 = UR3(ur3Base);

            % Adding UR5 robot
            %obj.robotUR5 = LinearUR5e(ur5Base);

            % Adding robotFetch
            obj.robotFetch = FetchRobot(fetchBase,obj.workspace);
                  
            % Add Table, Cones, Walls
            obj.environment = EnvironmentSetUp(obj.workspace, 2.25, centerpnt);
            

            % Adding bricks
%             for i = 1:1:brickNum
%                 obj.bricks(i) = Brick(obj.brickX(i),obj.brickY(i),obj.brickZ(i),0); % must fix up adding poses
%                 %obj.bricks(i).brickWallIndex = i; 
%             end
        end
        
        function teaching(obj)
            obj.robotFetch.teaching();
        end

        function addObject(obj, brickNum, object)
            brickX = object.X;
            brickY = object.Y;
            brickZ = object.Z;
            obj.brickX = brickX; obj.brickY = brickY; obj.brickZ = brickZ;
            obj.brickNum = brickNum;

            for i = 1:1:brickNum
                obj.bricks(i) = Brick(obj.brickX(i),obj.brickY(i),obj.brickZ(i),0); % must fix up adding poses
                %obj.bricks(i).brickWallIndex = i; 
            end

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

        function Recycle(obj, robot)
            
            %headCamera = RobotTemplateClass(getenv("ROS_IP"));

            switch robot.currentState
                
                case obj.LocateObject
                    if (robot.currentState==obj.LocateObject)
                        obj.headCamera.perceptionFcn(obj.headCamera.subPoint.LatestMessage, obj.headCamera.subCloud.LatestMessage);
                        obj.objectLocation = obj.headCamera.targetPoint;
    
                        robot.currentState = obj.MoveToObject; 
                        robot.previousState = obj.LocateObject; 

                        obj.objectCarteason = [obj.objectLocation.X obj.objectLocation.Y obj.objectLocation.Z];
                        disp("Object Located at: ");
                        display(obj.objectCarteason);
                    end

                case obj.MoveToObject
                    if (robot.currentState==obj.MoveToObject && robot.previousState==obj.LocateObject)
                        
                        qCurrent = robot.model.getpos; 
                        % objectCarteason = [obj.objectLocation.X obj.objectLocation.Y obj.objectLocation.Z];
                        poseObject = transl(obj.objectCarteason);
                                                
                        qBrick= robot.model.ikcon(poseObject, qCurrent);
                        robot.armTraj = jtraj(qCurrent, qBrick, obj.trajSteps);


                        robot.previousState = obj.MoveToObject;  % why here?
                        robot.steps = 1;
                    
                        if robot.steps <= obj.trajSteps
                            for i = 1:1:obj.trajSteps
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %result(robot.steps) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                robot.model.animate(robot.armTraj(robot.steps,:));
                                robot.steps = robot.steps + 1;
                            end
                            robot.currentState = obj.MoveToBin;
                        else 
                            robot.currentState = obj.MoveToBin;
                            robot.previousState = obj.MoveToObject; %duplicate?
                            robot.steps = 1;
                        end
                    end

                case obj.MoveToBin
                    if (robot.currentState==obj.MoveToBin && robot.previousState==obj.MoveToObject)

                        %% Move to way points to be above bin
                           disp("Moving Object to Recycling Bin");
                        %% let go
                           disp("Dropping Object");
                        %% scan for another object?
                        
                        
                    end
                    robot.taskcompleted = true;


            end

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
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %result(robot.steps) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    end
end

