classdef JointController
    %SANITISE
    
    properties
        q=[]; 
        GoalMsg;
        Arm; 
    end
    
    methods
        function obj = JointController()

            %jointSub = rossubscriber('/joint_states');
            %jntState = receive(jointSub);
            
            %Set up the action server
            [obj.Arm, obj.GoalMsg] = rosactionclient('arm_controller/follow_joint_trajectory');
            waitForServer(obj.Arm);
            
            %Label the joint names for the fetch robot

            obj.GoalMsg.Trajectory.JointNames = {   'torso_lift_link',...
                                                'shoulder_pan_joint',...
                                                'shoulder_lift_joint',...
                                                'upperarm_roll_joint',...
                                                'elbow_flex_joint',...
                                                'forearm_roll_joint',...
                                                'wrist_flex_joint',...
                                                'wrist_roll_joint'};
            
            

        end
        
        function SendTraj(obj, jntState)

            
            %Get the current joint states for the joints
%             numJoints = size(obj.GoalMsg.Trajectory.JointNames, 2); 
%             idx = ismember(jntState.Name, obj.GoalMsg.Trajectory.JointNames);
%             curPos = jntState.Position(idx);   
            curPos = jntState;
            
            %Make a point for the robot to go to                                
            trajPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            trajPoint1.Positions = curPos;
            trajPoint1.Positions(1) = 0;
            trajPoint1.Velocities = zeros(1, 8);
            trajPoint1.TimeFromStart = rosduration(1.0);
            
            %Execute the action
            sendGoalAndWait(obj.Arm, obj.GoalMsg);
            
            %rosshutdown;
            % Package ROS message and send to the robot
            %obj.PubMsg.X = obj.ControlOutputs;
            %send(obj.MyPub,obj.PubMsg);
        end
    end
end

