classdef appControl
    %APPCONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %sim; 

        taskCompleted = false; %Boolean for task completed 
        workspace = [-0.2 1.7 -1.2 1.2 0.2 1.5]; % workspace for matlab simulation
        centerpnt = [1,0,0.25];  % Centre point for table proxy in matlab
        binPoint = [-0.2,0.5,0]; % bin centre point 
        
        % estop functionality
        eStop = false; 
        
        % Add starting Poses for Fetch Robot
        fetchBase = transl(0,0,0.7) *trotz(pi);

        headCamera;
        simulation;
        object;

    end
    
    methods
        function obj = appControl()
            %APPCONTROL Construct an instance of this class
            %   Detailed explanation goes here

            % clf; clc; clear all;
            
            % Creates object conenct to ROS node for head camera inform                              ation
            obj.headCamera = RobotTemplateClass(getenv("ROS_IP"));
            
            % Generate the Simulation
            obj.simulation = Simulation(obj.fetchBase, obj.workspace, obj.centerpnt, obj.binPoint, obj.headCamera);
            
            % calling percetion 
            %system('cd /home/ian/git/uts-snc-and-robotics-fetchbot/fetchbot_src/build && rosrun fetchbot perception')
            %pause(5)
            % Grab object location from camera in ROS
            obj.headCamera.perceptionFcn(obj.headCamera.subPoint.LatestMessage, obj.headCamera.subCloud.LatestMessage);
            obj.object = obj.headCamera.targetPoint;
            % add object to matlab simulation
            obj.simulation.addObject(1,obj.object);

            currentPos = obj.simulation.robotFetch.model.getpos;

            if currentPos ~= obj.simulation.robotFetch.qHome
                disp("Resetting Arm");
                qHome = obj.simulation.robotFetch.qHome;
                obj.simulation.MoveArm(qHome);

            end

            

        end


        function Recycle(obj)

            while (obj.taskCompleted == false)
            obj.simulation.Recycle();
            
                if (obj.simulation.robotFetch.taskcompleted == true)
                    obj.taskCompleted = true;
                    disp("Object Recycled");
                end
            end

        end

        

    end
end

