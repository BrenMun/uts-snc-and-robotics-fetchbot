classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %>Workspace 
        workspace = [-4 4 -4 4 -1.2 2.5];
        scale = 0.2; 
        
        %> Flag to indicate if gripper is used
        useGripper = false;     

        % For calculations
        maxReachUR3X;
        maxReachUR3Y;
        maxReachUR3Z;
        volumeUR3;
        qlim; 
        endEffectorPoses;
        q;
        base;

        % State machine
        
        armTraj;
        taskcompleted = false;
        previousState = 0; 
        currentState = 0; 
        targetedBrickID = 0;
        steps = 0; 
        qHome = [pi, 0, 0, 0, 0, 0];
       

    end


    
    methods%% Class for UR3 robot simulation
function self = UR3Gripper(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace
        
% robot = 
self.GetUR3Robot();

% robot = 
self.PlotAndColourRobot();%robot,workspace);

end
%% Get and plot Robot
function self = UR3(baseUR3)
    
    % Set up UR3 robot workspace        
    %> robot = 
    self.GetUR3Robot();

    %> Define the base location
    self.model.base = baseUR3;
    self.base = baseUR3;

    %> robot = 
    self.PlotLinksUR3(); % only for stick model
end

%% GetUR3Robot
% Given a name (optional), create and return a UR5 robot model
function GetUR3Robot(self)
    pause(0.001);
    name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];

    L1 = Link('d',0.1519 ,'a',0       ,'alpha', pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L2 = Link('d',0      ,'a',-0.24365,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-180),deg2rad(0)]);
    L3 = Link('d',0      ,'a',-0.21325,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L4 = Link('d',0.11235,'a',0       ,'alpha', pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L5 = Link('d',0.08535,'a',0       ,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L6 = Link('d',0.0819 ,'a',0       ,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);   

    % UR3 DH parameters found at: 
    % https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    
    self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end

%% 
function PlotLinksUR3(self)
    self.model.plot(self.qHome,'workspace',self.workspace,'scale',self.scale, 'noarrow'); 
    hold on
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end 
%% Calculate the arms max reach and volume

% Reach
% Calculat max reach via teach (qMax). 
% Animate moving to that position

function CalculateMaxReach(self, baseUR3)

disp('Please press Enter to continue');
pause(); 
disp('UR3 Max Reach X axis');
    for i =0:-pi/10:-pi/2
        self.model.plot([pi, 0, 0, i, 0, 0]);
    end
    self.q =[pi, 0, 0, -pi/2, 0, 0];
    maxReachKineX = self.model.fkine(self.q);
    maxReachUR3X = ([abs(maxReachKineX(1,4) - baseUR3(1,4)); 
                     abs(maxReachKineX(2,4) - baseUR3(2,4)); 
                     abs(maxReachKineX(3,4) - baseUR3(3,4))]);

    self.maxReachUR3X = maxReachUR3X;

    disp('Please press Enter to continue');
    pause(); 
    disp('UR3 Max Reach Y axis');
    
    for i =pi:pi/10:(3*pi/2)
        self.model.plot([i, 0, 0, -pi/2, 0, 0]);
    end
    self.q =[(3*pi/2), 0, 0, -pi/2, 0, 0];
    maxReachKineY = self.model.fkine(self.q);
    maxReachUR3Y = ([abs(maxReachKineY(1,4) - baseUR3(1,4)); 
                     abs(maxReachKineY(2,4) - baseUR3(2,4)); 
                     abs(maxReachKineY(3,4) - baseUR3(3,4))]);
    
    self.maxReachUR3Y = maxReachUR3Y;

    disp('Please press Enter to continue');
    pause(); 
    disp('UR3 Max Reach Z axis');

    for i =0:-pi/10:-pi/2
        self.model.plot([(3*pi/2), i, 0, -pi/2, 0, 0]);
    end
    self.q =[(3*pi/2), -pi/2, 0, -pi/2, 0, 0];
    maxReachKineZ = self.model.fkine(self.q);
    maxReachUR3Z = ([abs(maxReachKineZ(1,4) - baseUR3(1,4)); 
                     abs(maxReachKineZ(2,4) - baseUR3(2,4)); 
                     abs(maxReachKineZ(3,4) - baseUR3(3,4))]);

    self.maxReachUR3Z = maxReachUR3Z;

    disp('Please press Enter to continue');
    pause(); 

    self.model.plot(self.qHome);

    disp ('Max Arm Reach from UR3 Base (x,y,z) = ')
    display(self.maxReachUR3X(1)), display(self.maxReachUR3Y(2)), display(self.maxReachUR3Z(3))
end

%%
function UR3ReachVolume(self)
    % Cycle through models Joint limits
    self.qlim = self.model.qlim();
    counter = 1;
    for q1 = self.qlim(1,1):(pi/10):self.qlim(1,2)
        for q2 = self.qlim(2,1):(pi/10):self.qlim(2,2)
            endEff = self.model.fkine([q1, q2, 0, -pi/2, 0, 0]);
            self.endEffectorPoses(counter,:) = endEff(1:3,4)';
            counter = counter + 1;
        end
    end
    % Plot poses of end effectors
    % https://au.mathworks.com/help/matlab/ref/plot3.html
    volume = plot3(self.endEffectorPoses(:,1), ...
                self.endEffectorPoses(:,2), ...
                self.endEffectorPoses(:,3));
                
    % https://au.mathworks.com/help/matlab/ref/convhull.html#mw_f9cda693-9c7a-40a0-84e0-7cee7b2af88b
    [~, self.volumeUR3] = convhull(self.endEffectorPoses(:,1), ...
                                   self.endEffectorPoses(:,2), ...
                                   self.endEffectorPoses(:,3));
    disp ('UR3 Arm Reach Volume = ')
    display (self.volumeUR3)
    disp('Please press Enter to continue');
    pause(); 
    
    set(volume, 'Visible', 'off');
end
    end
end