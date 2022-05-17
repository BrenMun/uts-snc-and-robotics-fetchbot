classdef JointController
    %SANITISE
    properties
        q=[]; 
        GoalMsg;
        Arm;
        gripAct;
        gripGoal;
    end
    
    methods
        % https://au.mathworks.com/help/robotics/ug/control-pr2-arm-movements-using-actions-and-ik.html
        function obj = JointController()
            %jointSub = rossubscriber('/joint_states');
            %jntState = receive(jointSub);
            
            %Set up the action server
            [obj.Arm, obj.GoalMsg] = rosactionclient('arm_with_torso_controller/follow_joint_trajectory');
            waitForServer(obj.Arm);
            
            %Label the joint names for the fetch robot
            obj.GoalMsg.Trajectory.JointNames = {
                'torso_lift_joint',...
                'shoulder_pan_joint',...
                'shoulder_lift_joint',...
                'upperarm_roll_joint',...
                'elbow_flex_joint',...
                'forearm_roll_joint',...
                'wrist_flex_joint',...
                'wrist_roll_joint'
            };
            
            %Set up the gripper action server
            [obj.gripAct,obj.gripGoal] = rosactionclient('gripper_controller/gripper_action');
            waitForServer(obj.gripAct);
        end
        
        function SendTraj(obj, p1,p2)
    
            % Point 1
            tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint1.Positions = p1;        % zeros(1,8);
            tjPoint1.Velocities = zeros(1,8);
            tjPoint1.TimeFromStart = rosduration(1.0);
            
            % Point 2
            tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            tjPoint2.Positions = p2;         %[-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
            tjPoint2.Velocities = zeros(1,8);
            tjPoint2.TimeFromStart = rosduration(2.0);

            %Execute the action
            obj.GoalMsg.Trajectory.Points = [tjPoint1,tjPoint2];
            sendGoalAndWait(obj.Arm, obj.GoalMsg);
            
            %rosshutdown;
            % Package ROS message and send to the robot
            %obj.PubMsg.X = obj.ControlOutputs;
            %send(obj.MyPub,obj.PubMsg);
        end

        function grip(obj, position)
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            %gripperCommand.Position = 0.0;  
            gripperCommand.Position = position;
            obj.gripGoal.Command = gripperCommand;
            sendGoalAndWait(obj.gripAct,obj.gripGoal);
        end
    end
end

