classdef HeadCamera
    properties
        % ROS publishers and subscribers
        subImage = []; 
        subCloud = [];
        % Current time
        CurrentTime = 0;        
        % Tranformation Tree
        tfTree;        
        % Object carteason
        targetPoint = [];
        % action client variables
        head; headGoal;
    end
    
    methods
        function obj = HeadCamera(ipAddr)
            % Connect to ROS master at the IP address specified
            rosshutdown; rosinit(ipAddr);
            % Subscribes to target_point
            obj.subImage = rossubscriber('/head_camera/rgb/image_raw','sensor_msgs/Image');
            % Subscribe to camera point cloud
            obj.subCloud = rossubscriber('/head_camera/depth_registered/points', 'sensor_msgs/PointCloud2');
            % Transform Tree Object
            obj.tfTree = rostf;
            %Set up the head action server
            [obj.head,obj.headGoal] = rosactionclient('/head_controller/point_head');
            waitForServer(obj.head);
        end     
        
        function Look(obj)
            lookCommand = rosmessage('control_msgs/PointHeadActionGoal');
            
            % PointStamped
            lookCommand.Goal.Target.Point.X = 1;
            lookCommand.Goal.Target.Point.Y = 0;
            lookCommand.Goal.Target.Point.Z = 0;
            lookCommand.Goal.Target.Header.FrameId ='base_link';

            % pointing frame
            lookCommand.Goal.PointingFrame = 'high_def_frame';
            lookCommand.Goal.PointingAxis.X = 1;
            lookCommand.Goal.PointingAxis.Y = 0;
            lookCommand.Goal.PointingAxis.Z = 0;
            
            % duration
            lookCommand.Goal.MinDuration = rosduration(0.5);
            lookCommand.Goal.MaxVelocity = 1.0;
            obj.headGoal = lookCommand;
            sendGoalAndWait(obj.head,obj.headGoal);
        end

        function targetPoint = getTargetPoint(obj)
            img = readImage(obj.subImage);                                  %convert msg to image
            hsvImg = rgb2hsv(img);                                          %convert rgb to hsv
            hsvMin = [28,0,107];                                            %min hsv
            hsvMax = [256,256,256];                                         %max hsv
            % hsv mask
            BW = ((hsvImg(:,:,1) >= hsvMin(1)) | (hsvImg(:,:,1) <= hsvMax(1))) & ...
                  (hsvImg(:,:,2) >= hsvMin(2)) & (hsvImg(:,:,2) <= hsvMax(2)) & ...
                  (hsvImg(:,:,3) >= hsvMin(3)) & (hsvImg(:,:,3) <= hsvMax(3));
            maskedImg = img;                                                %masked image
            maskedImg(repmat(~BW, [1 1 3])) = 0;                            %filter out hsv
            imshow(maskedImg);                                              %show image
        end
    end
end

