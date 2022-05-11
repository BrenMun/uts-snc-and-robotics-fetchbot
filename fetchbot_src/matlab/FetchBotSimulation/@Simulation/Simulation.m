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
        steps; % for interpolation

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
                    drawnow(); 
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
                    drawnow();
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
                    drawnow();
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

        function moveThroughWaypoints(obj)
            %waypoint = [0    0.9346    0.0972    0.1257   -2.0000   -0.1258   -0.5700    0.0012];
            %obj.addWayPoint(obj.robotFetch, waypoint);
            %pause(2);
            
            waypoint1 = [0    0.9346    0.0698    1.2567   -0.9643    0.2512   -0.8754    0.001];
            %obj.addWayPoint(obj.robotFetch, waypoint);
            %pause(2);
            waypoint2 = [0    0.9346    0.0698    1.5080   -0.3339    0.1255   -2.0972    1.2579];
            %obj.addWayPoint(obj.robotFetch, waypoint);

            obj.MoveArm(obj.robotFetch, waypoint1);
            pause(1);
            obj.MoveArm(obj.robotFetch, waypoint2);

        end

        function MoveArm(obj, robot, qEnd)

            %  Go through until there are no step sizes larger than 1 degree using diff(rad2deg(jtraj(q1,q2,steps))
            qCurrent = robot.model.getpos;
            q2 = qEnd;
            qWaypoints = [qCurrent, q2];
            qMatrix = obj.FineInterpolation(qCurrent, q2, 0.5);
            %qMatrix = obj.InterpolateWaypointRadians(qWaypoints,deg2rad(5));

            robot.steps = 1;

            result = true(obj.steps,1); % create logical vecter for results
        
            %if robot.steps <= obj.trajSteps
                %for i = 1:1:obj.trajSteps
                for i = 1: obj.steps
               
                    result(i) = IsCollision(robot,qMatrix(i,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                    
                    robot.model.animate(qMatrix(i,:));
                    drawnow();
                    robot.steps = robot.steps + 1;
                end 
                %robot.steps = 1;
                obj.JointController.SendTraj(qCurrent,q2);
                %pause(1);
            %end

        end


        function Recycle(obj, robot)
            disp("Starting to Recycle");
            
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
                        obj.objectCarteason

                    end 

                case obj.MoveToObject
                    if (robot.currentState==obj.MoveToObject && robot.previousState==obj.LocateObject)
                        disp("Moving to collect object");

                        % move through way points
                        obj.moveThroughWaypoints();

                        qCurrent = robot.model.getpos; 
                        % objectCarteason = [obj.objectLocation.X obj.objectLocation.Y obj.objectLocation.Z];+
                        
                        offsetObject = [obj.objectCarteason(1)+0.01,... %% making offset to graps object
                                        obj.objectCarteason(2),...
                                        obj.objectCarteason(3)+0.3];

                        %poseObject = transl(obj.objectCarteason);
                        poseObject = transl(offsetObject)* troty(pi);
                                                
                        qObject= robot.model.ikcon(poseObject, qCurrent);
                        %robot.armTraj = jtraj(qCurrent, qObject, obj.trajSteps);


                        robot.previousState = obj.MoveToObject;  % why here?
                        robot.steps = 1;

                        obj.MoveArm(robot, qObject)

                        robot.currentState = obj.MoveToBin;
                        robot.steps = 1;
                    
%                         if robot.steps <= obj.trajSteps
%                             for i = 1:1:obj.trajSteps
%                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 result(i) = IsCollision(robot,robot.armTraj(robot.steps,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
%                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 %qCurrent = robot.model.getpos;
%                                 %obj.JointController.SendTraj(qCurrent,robot.armTraj(i,:));
%                                 
%                                 robot.model.animate(robot.armTraj(robot.steps,:));
%                                 
%                                 robot.steps = robot.steps + 1;
%                             end
%                             obj.JointController.SendTraj(qCurrent,qObject);
%                             robot.currentState = obj.MoveToBin;
%                         else 
%                             robot.currentState = obj.MoveToBin;
%                             robot.previousState = obj.MoveToObject; %duplicate?
%                             robot.steps = 1;
%                         end
                        
                    end

                case obj.MoveToBin
                    if (robot.currentState==obj.MoveToBin && robot.previousState==obj.MoveToObject)
    
                        % Wait until message the object has been grasped
                        % Move to way points to be above bin
                           disp("Moving Object to Recycling Bin");
                           % move arm through way points
                        % let go
                           disp("Dropping Object");
                           obj.grip(1.0);
                        % scan for another object?

                        
                        
                        
                    end
                    robot.taskcompleted = true;


            end

        end
%%
%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(obj,q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
%disp("Step count");
obj.steps = steps;
steps;
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(obj, waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; obj.FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end


%%        
    end
end

