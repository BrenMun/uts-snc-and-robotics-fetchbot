classdef RobotTemplateClass < handle
    properties
        % ROS publishers and subscribers
        subPoint = []; subCloud = []; MyPub = []; PubMsg = [];
        % Perception algorithm   
        PerceptionParam = 1;
        % Control algorithm
        ControlInputs = 0; ControlOutputs = 0; ControlParams = struct('gain',1);
        % Current time
        CurrentTime = 0;
    end
    
    methods       
        %% CONSTRUCTOR
        function obj = RobotTemplateClass(ipAddr)
            % Connect to ROS master at the IP address specified
            rosshutdown; rosinit(ipAddr);
            % Subscribes to target_point
            obj.subPoint = rossubscriber('/target_point','geometry_msgs/PointStamped');
            % Subscribe to camera point cloud
            obj.subCloud = rossubscriber('/head_camera/depth_registered/points', 'sensor_msgs/PointCloud2');
            % Create publisher (no publisher yet)
             %[obj.MyPub,obj.PubMsg] = rospublisher('/my_pub_topic','geometry_msgs/Point'); 
        end     
        
        %% PERCEPTION ALGORITHM
        % Receives sensor data, runs perception algorithm
        function perceptionFcn(obj,receivedPointStamped,receivedCloud)           
            if isempty(receivedPointStamped)
                disp 'message is empty'
            else
                obj.ControlInputs = myPerceptionAlgorithm(...
                    receivedPointStamped.Point,...
                    receivedCloud,...
                    obj.PerceptionParam...
                );
            end
        end
        
        %% CONTROL ALGORITHM
        % Runs control algorithm and publishes ROS commands
        function controlFcn(obj)            
            % Control algorithm
            obj.ControlOutputs = myControlAlgorithm(obj.ControlInputs,obj.ControlParams);
            % Package ROS message and send to the robot
            obj.PubMsg.X = obj.ControlOutputs;
            send(obj.MyPub,obj.PubMsg);
        end       
    end
    
end