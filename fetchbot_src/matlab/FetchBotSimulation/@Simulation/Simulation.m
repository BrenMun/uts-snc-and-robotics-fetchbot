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

        JointController
        
       

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
            axis equal;

%             side = 1;
%             plotOptions.plotFaces = true;
%             [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
%             axis equal
%             camlight
%             obj.tableFaces = faces;
%             obj.tableVertices = vertex; 
%             obj.tableFaceNormals = faceNormals; 

            % Adding class for sending messages to ROS
            obj.JointController = JointController;
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

        function getPos(obj)
            q = obj.robotFetch.model.getpos;
            disp(q); 
        end

        function TestMovement(obj, robot)

            disp("test movement");
            qCurrent = robot.model.getpos; 

            %poseObject = transl(obj.objectCarteason);               
            %qObject= robot.model.ikcon(poseObject, qCurrent);
            qObject = [0    0.8219   -0.0780   -0.8516   -1.2403    1.0248   -1.4547    2.6003];

            robot.armTraj = jtraj(qCurrent, qObject, obj.trajSteps);


            robot.previousState = obj.MoveToObject;  % why here?
            robot.steps = 1;
        
            if robot.steps <= obj.trajSteps
                for i = 1:1:obj.trajSteps
               
                    %qCurrent = robot.model.getpos;
                    %obj.JointController.SendTraj(qCurrent,robot.armTraj(i,:));
                    result(i) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                    robot.model.animate(robot.armTraj(robot.steps,:));
                    robot.steps = robot.steps + 1;
                end 
                robot.steps = 1;
                obj.JointController.SendTraj(qCurrent,qObject);
            end
            
        end

        function resetRobot(obj, robot)

            disp("Resetting");
            qCurrent = robot.model.getpos; 

            %poseObject = transl(obj.objectCarteason);               
            %qObject= robot.model.ikcon(poseObject, qCurrent);
            qObject = robot.qHome;

            robot.armTraj = jtraj(qCurrent, qObject, obj.trajSteps);


            robot.previousState = obj.MoveToObject;  % why here?
            robot.steps = 1;
        
            if robot.steps <= obj.trajSteps
                for i = 1:1:obj.trajSteps
               
                    %qCurrent = robot.model.getpos;
                    %obj.JointController.SendTraj(qCurrent,robot.armTraj(i,:));
                    result(i) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                    robot.model.animate(robot.armTraj(robot.steps,:));
                    robot.steps = robot.steps + 1;
                end 
                robot.steps = 1;
                obj.JointController.SendTraj(qCurrent,qObject);
            end
            
        end

        function addWayPoint(obj, robot, waypoint)

            qCurrent = robot.model.getpos; 
            qObject = waypoint;

            robot.armTraj = jtraj(qCurrent, qObject, obj.trajSteps);

            robot.previousState = obj.MoveToObject;  % why here?
            robot.steps = 1;
        
            if robot.steps <= obj.trajSteps
                for i = 1:1:obj.trajSteps
               
                    result(i) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                    
                    robot.model.animate(robot.armTraj(robot.steps,:));
                    robot.steps = robot.steps + 1;
                end 
                robot.steps = 1;
                obj.JointController.SendTraj(qCurrent,qObject);
            end
            
        end
        
        function checkCollisions(obj, robot)
            q = obj.robotFetch.model.getpos;
            result = IsCollision(robot,q,obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
            if result == 1
                disp(['Intersection at step ', num2str(i)]);
                q;
                p3.fkine(q)
                
            else
                disp('No intersection');
                result
            end
        end

        function grip(obj, position)
            obj.JointController.grip(position);
        end

       

        function Recycle(obj, robot)
            disp("Starting to Recycle");
            %robot.currentState=obj.LocateObject;
            
            %headCamera = RobotTemplateClass(getenv("ROS_IP"));

            switch robot.currentState
                
                case obj.LocateObject
                    if (robot.currentState==obj.LocateObject)
                        
                        obj.headCamera.perceptionFcn(obj.headCamera.subPoint.LatestMessage, obj.headCamera.subCloud.LatestMessage);
                        obj.objectLocation = obj.headCamera.targetPoint;
    
                        robot.currentState = obj.MoveToObject; 
                        robot.previousState = obj.LocateObject; 

                        obj.objectCarteason = [obj.objectLocation.X obj.objectLocation.Y obj.objectLocation.Z+0.05];
                        disp("Object Located at: ");
                        display(obj.objectCarteason);

                    end 

                case obj.MoveToObject
                    if (robot.currentState==obj.MoveToObject && robot.previousState==obj.LocateObject)
                        disp("Moving to collect object");
                        qCurrent = robot.model.getpos; 
                        % objectCarteason = [obj.objectLocation.X obj.objectLocation.Y obj.objectLocation.Z];
                        poseObject = transl(obj.objectCarteason);
                                                
                        qObject= robot.model.ikcon(poseObject, qCurrent);
                        robot.armTraj = jtraj(qCurrent, qObject, obj.trajSteps);


                        robot.previousState = obj.MoveToObject;  % why here?
                        robot.steps = 1;
                    
                        if robot.steps <= obj.trajSteps
                            for i = 1:1:obj.trajSteps
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                result(i) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %qCurrent = robot.model.getpos;
                                %obj.JointController.SendTraj(qCurrent,robot.armTraj(i,:));
                                
                                robot.model.animate(robot.armTraj(robot.steps,:));
                                robot.steps = robot.steps + 1;
                            end
                            obj.JointController.SendTraj(qCurrent,qObject);
                            robot.currentState = obj.MoveToBin;
                        else 
                            robot.currentState = obj.MoveToBin;
                            robot.previousState = obj.MoveToObject; %duplicate?
                            robot.steps = 1;
                        end
                    end

                case obj.MoveToBin
                    if (robot.currentState==obj.MoveToBin && robot.previousState==obj.MoveToObject)

                        % Move to way points to be above bin
                           disp("Moving Object to Recycling Bin");
                        % let go
                           disp("Dropping Object");
                        % scan for another object?

                        % So i need to build waypoints like LAB5
                        
                        
                    end
                    robot.taskcompleted = true;


            end

        end



%%        
    end
end

