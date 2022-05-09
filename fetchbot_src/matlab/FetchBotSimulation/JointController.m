classdef JointController
    %SANITISE
    
    properties
        q=[]; 
        GoalMsg;
        Arm; 
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

            obj.GoalMsg.Trajectory.JointNames = {'torso_lift_joint',...
                                                'shoulder_pan_joint',...
                                                'shoulder_lift_joint',...
                                                'upperarm_roll_joint',...
                                                'elbow_flex_joint',...
                                                'forearm_roll_joint',...
                                                'wrist_flex_joint',...
                                                'wrist_roll_joint'};
            
    
        end
        
        function SendTraj(obj, p1,p2)

            %Get the current joint states for the joints
%             numJoints = size(obj.GoalMsg.Trajectory.JointNames, 2); 
%             idx = ismember(jntState.Name, obj.GoalMsg.Trajectory.JointNames);
%             curPos = jntState.Position(idx);   
%             %curPos = jntState;
%             
%             %Make a point for the robot to go to                                
%             trajPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%             trajPoint1.Positions = curPos;
%             trajPoint1.Positions(1) = 0;
%             trajPoint1.Velocities = zeros(1, 8);
%             trajPoint1.TimeFromStart = rosduration(1.0);
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

            obj.GoalMsg.Trajectory.Points = [tjPoint1,tjPoint2];

            
            %Execute the action
            sendGoalAndWait(obj.Arm, obj.GoalMsg);
            
            %rosshutdown;
            % Package ROS message and send to the robot
            %obj.PubMsg.X = obj.ControlOutputs;
            %send(obj.MyPub,obj.PubMsg);
        end

        function sendTrajMsgs(obj, pos1, pos2 )  % https://au.mathworks.com/matlabcentral/answers/344938-simulink-ros-publishing-jointtrajectory-messages
          
          % Create the publisher only on the first function call
          persistent pub
          if isempty(pub)
              pub = rospublisher('/my_joints','arm_controller/follow_joint_trajectory');
          end
          
          % Create an empty message with the joint names
          msg = rosmessage('arm_controller/follow_joint_trajectory');
          msg.JointNames = {'joint1','joint2','joint3'};
          
          % Fill in the message positions
          p1 = rosmessage('arm_controller/follow_joint_trajectory');
          p1.Positions = pos1;
          msg.Points(1) = p1;
          p2 = rosmessage('arm_controller/follow_joint_trajectory');
          p2.Positions = pos2;
          msg.Points(2) = p2;
          
          % Send messages
          send(pub,msg)
        end
    end
end
