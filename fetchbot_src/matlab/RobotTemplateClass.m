classdef RobotTemplateClass < handle
    properties
        % ROS publishers and subscribers
        MySub = []; MyPub = []; PubMsg = [];
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
            % Create subscriber (subscribes to target_point of type geometry_msgs::PointStamped)
            obj.MySub = rossubscriber('/target_point','geometry_msgs/PointStamped');
            % Create publisher (no publisher yet)
             %[obj.MyPub,obj.PubMsg] = rospublisher('/my_pub_topic','geometry_msgs/Point'); 
        end     
        
        %% PERCEPTION ALGORITHM
        % Receives sensor data, runs perception algorithm
        function perceptionFcn(obj,receivedMsg)           
            if isempty(receivedMsg)
                disp 'message is empty'
            else
                receivedData = receivedMsg.Point;
                obj.ControlInputs = myPerceptionAlgorithm(receivedData,obj.PerceptionParam);  
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