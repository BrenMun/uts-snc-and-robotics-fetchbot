classdef Simulation < handle % Passes by reference
    %SIMULATION This class sets up the simulation in Matlab
    %   This class when constructed sets up the environment in matlab
    
    properties
        % Handle
        simulation

        %Objects
        %robotUR3;
        %robotUR5;
        robotFetch;
        environment;
        bricks = Brick.empty;
        objectNum;
        workspace;
        binPoint = []; 
        objectLocation;
        headCamera;
        trash_bin;

        % Variables
        %tableTolerance = 0.5;
        %brickWallPoses= zeros(9,3); 
        %brickWallIndex = 1; 
        %ur3Base; 
        %ur5Base;
        objectX; 
        objectY; 
        objectZ;
        %trajSteps = 50; 
        %wallX = [];
        %wallY = [];
        %wallZ = [];
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
%         LocateNextBrick = 0; 
%         PickUpBrick = 1; 
%         PlaceBrick = 2; 
%         ReturnHome = 3; 
%         Uknown = 4; 

%         % Brick states for State Machine
%         state = 0; 
%         unmoved = 0;
%         targeted = 1; 
%         moving = 3; 
%         moved = 4; 
%         unknown = 5;

    end

    
    methods
        %% Construct an instance of this class
        function obj = Simulation(fetchBase, workspace, centerpnt,binPoint, headCamera)
            % adding variables from main to global variables in simulations
            obj.workspace = workspace; 
            obj.binPoint = binPoint; 
            obj.headCamera = headCamera;
            % Adding robotFetch as object
            obj.robotFetch = FetchRobot(fetchBase,obj.workspace);
            % Setting up the environment
            obj.environment = EnvironmentSetUp(obj.workspace, 2.25, centerpnt);
            axis equal;

            obj.trash_bin = Trash_bin(0.0,0.20,-1.1); %YZ AXIS FLIPPED,(X = 0, Y = -1.1, Z = 0.2)
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
            view(3);
        end
        
        function teaching(obj) % simple function that calls teach
            obj.robotFetch.teaching();
        end

        function addObject(obj, objectNum, object) % This function adds the object to the environment, getting its coordinates from ROS
            brickX = object.X;
            brickY = object.Y;
            brickZ = object.Z;
            obj.objectX = brickX; obj.objectY = brickY; obj.objectZ = brickZ;
            obj.objectNum = objectNum;
            for i = 1:1:objectNum
                obj.bricks(i) = Brick(obj.objectX(i),obj.objectY(i),obj.objectZ(i),0); % must fix up adding poses
                %obj.bricks(i).brickWallIndex = i; 
            end
        end

        function getPos(obj) % simple function to get pos in main
            q = obj.robotFetch.model.getpos;
            disp(q); 
        end

        function checkCollisions(obj, robot) % simple function to check collisions from main
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

        function grip(obj, position) % this function sends a message to ros to graps 
            obj.JointController.grip(position);
        end

        function moveThroughWaypoints(obj) % This function contains some way points to avoid the table
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
            %obj.MoveArm(obj.robotFetch, waypoint2);
        end

        function MoveArm(obj, robot, qEnd) % function connects to ros and moves the arm based on matlab movements
            %  Go through until there are no step sizes larger than 1 degree using diff(rad2deg(jtraj(q1,q2,steps))
            qCurrent = robot.model.getpos;
            q2 = qEnd;
            %qWaypoints = [qCurrent, q2];
            qMatrix = obj.FineInterpolation(qCurrent, q2, 0.5);
            %qMatrix = obj.InterpolateWaypointRadians(qWaypoints,deg2rad(5));
            robot.steps = 1;
            result = true(obj.steps,1); % create logical vecter for results
            for i = 1: obj.steps
                result(i) = IsCollision(robot,qMatrix(i,:),obj.environment.tableFaces, obj.environment.tableVertices, obj.environment.tableFaceNormals ,false);
                robot.model.animate(qMatrix(i,:));
                drawnow();
                robot.steps = robot.steps + 1;
            end 
            obj.JointController.SendTraj(qCurrent,q2);
        end

        function Recycle(obj, robot) % case maschine to locate, graps and move an object to the bin
            disp("Starting to Recycle");
            %headCamera = RobotTemplateClass(getenv("ROS_IP"));
            switch robot.currentState
                case obj.LocateObject % This case locates the object through the information found in ros. 
                    if (robot.currentState==obj.LocateObject)
                        % Run perception function
                        obj.headCamera.perceptionFcn(obj.headCamera.subPoint.LatestMessage, obj.headCamera.subCloud.LatestMessage);
                        % get the object's location
                        obj.objectLocation = obj.headCamera.targetPoint;
                        % get the current state
                        robot.currentState = obj.MoveToObject; 
                        % previous state
                        robot.previousState = obj.LocateObject; 
                        % XYZ of the object's position
                        obj.objectCarteason = [obj.objectLocation.X obj.objectLocation.Y + 0.01 obj.objectLocation.Z];
                        disp("Object Located at: ");
                        display(obj.objectCarteason);
                        obj.objectCarteason
                    end 

                case obj.MoveToObject % This case Moves the arm through a series of way points to avoid table, then moves to an offset of the objects location
                    if (robot.currentState==obj.MoveToObject && robot.previousState==obj.LocateObject)
                        %% Move Through Waypoints
                        disp("Moving to collect object");
                        obj.moveThroughWaypoints();                         % move through way points 
                        qCurrent = robot.model.getpos;                      % get current q
                        % offest of the target object's position
                        offsetObject = [obj.objectCarteason(1),...
                                        obj.objectCarteason(2),...
                                        obj.objectCarteason(3)+0.3];
                        poseObject = transl(offsetObject)* troty(pi);       % tf of object
                        qObject= robot.model.ikcon(poseObject, qCurrent);   % current joints of arm
                        robot.previousState = obj.MoveToObject;             % previous joints
                        robot.steps = 1;                                    % steps
                        qCurrent(1) = qObject(1);                           % change torso q value
                        obj.MoveArm(robot, qCurrent)                        % Move torso only to avoid collision
                        obj.MoveArm(robot, qObject)                         % Moving arm

                        %% Pick up object (RMRC)
                        d = 0.21;                                           %desired distance from end-effector to target
                        totalTime = 2;                                      %total time
                        deltaT = 0.02;                                      %frequency
                        p1 = poseObject(1:3,end);                           %initial point (last waypoint)
                        p2 = obj.objectCarteason;                           %target point (object)
                        q_guess = qObject;                                  %initial guess for joint angles (last q)
                        [qMatrix, posError, angleError] = ...               %return q to object, position error and angle error for plotting
                        solveRMRC(robot.model,p1,p2,q_guess,totalTime,deltaT,d);  %solve RMRC using parameters above
                        for i = 1:length(qMatrix(:,1))                      %plot trajectory
                            robot.model.animate(qMatrix(i,:));
                            drawnow();
                            robot.steps = robot.steps + 1;
                        end
                        obj.MoveArm(robot, qMatrix(end,:));                 %move arm to target
                        obj.grip(0.0);                                      %grasp object
                        prevQ = qMatrix(end,:)
                        obj.MoveArm(robot, [qObject(1) prevQ(2) ...         %lift object
                            prevQ(3) prevQ(4) prevQ(5) ...
                            prevQ(6) prevQ(7) prevQ(8)]);
                        % Setting states
                        robot.currentState = obj.MoveToBin;
                        robot.steps = 1;
                    end

                case obj.MoveToBin  % This case waits for the VS - RMRC grasping function to confirm it has grabbed the object, then moves through way points to the bin to deliver the package
                    if (robot.currentState==obj.MoveToBin && robot.previousState==obj.MoveToObject)
    
                        % Wait until message the object has been grasped
                            disp("Waiting for Graps Confirmation");
                            pause(1)
                            % wait for ros node to confirm movement
                        % Move to way points to be above bin
                           disp("Moving Object to Recycling Bin");
                           qCurrent = robot.model.getpos;
                           %qBin =[qCurrent(1)  1.3200   -0.1495   -0.1256   -0.8292   -0.1258    0.8263    0.0012];
                           qBin =[qCurrent(1)   1.4806   -0.0398    0.0000   -0.5591   -0.0001    1.5680    0.0012];
                           obj.MoveArm(robot, qBin);
                           
                        % let go
                           disp("Dropping Object");
                           obj.grip(1.0);
                        % scan for another object?
                        robot.previousState = obj.MoveToBin;

                        
                    end
                    robot.taskcompleted = true;
                    disp("Object Recycled - Returning Home");
                    robot.currentState = obj.LocateObject;

                    % Resets the robot to home
                    qHome = robot.qHome;
                    obj.MoveArm(robot, qHome);
                    pause(5); 
                    disp("Task Completed");

            end

        end




%% Decalring functions
%% FineInterpolation
% Calss jtraj until all step sizes aresmaller than a given max steps size
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

