classdef LinearUR5e < handle
    properties
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-2 2 -2 2 -1.2 2.5];
        scale = 0.2; 
        
        %> Flag to indicate if gripper is used
        useGripper = false;   
       
        % For calculations
        maxReachUR5X;
        maxReachUR5Y;
        maxReachUR5Z;
        volumeUR5;
        qlim; 
        endEffectorPoses;
        q;
        base;
        qHome = [0, 0, 0, 0, 0, 0, 0];

        % State machine
        
        armTraj;
        taskcompleted = false;
        previousState = 0; 
        currentState = 0; 
        targetedBrickID = 0;
        steps = 0; 


    end

    
    methods%% Class for UR5 robot simulation
function self = LinearUR5gripper(useGripper)
    self.useGripper = useGripper;
    
    %> Define the boundaries of the workspace
    
            
    %robot = 
    self.GetUR5Robot();
    %robot = 
    self.PlotAndColourRobot();%robot,workspace);
end
%%
function self = LinearUR5e(baseUR5)
    
    %> Define the boundaries of the workspace
            
    % robot = 
    self.GetUR5Robot();
    self.model.base = baseUR5;
    self.base = baseUR5;
    % robot = 
    self.PlotAndColourRobot();%robot,workspace);
end

%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetUR5Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LinearUR_5_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    % Create the UR5 model mounted on a linear rail
    L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
    L(2) = Link([0      0.1599  0       -pi/2   0]);
    L(3) = Link([0      0.1357  0.425   -pi     0]);
    L(4) = Link([0      0.1197  0.39243 pi      0]);
    L(5) = Link([0      0.093   0       -pi/2   0]);
    L(6) = Link([0      0.093   0       -pi/2	0]);
    L(7) = Link([0      0       0       0       0]);
    
    % Incorporate joint limits
    L(1).qlim = [-0.8 0];
    L(2).qlim = [-360 360]*pi/180;
    L(3).qlim = [-90 90]*pi/180;
    L(4).qlim = [-170 170]*pi/180;
    L(5).qlim = [-360 360]*pi/180;
    L(6).qlim = [-360 360]*pi/180;
    L(7).qlim = [-360 360]*pi/180;

    L(3).offset = -pi/2;
    L(5).offset = -pi/2;
    
    self.model = SerialLink(L,'name',name);
    
    % Rotate robot to the correct orientation
    self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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

function CalculateMaxReach(self, baseUR5)

    disp('Please press Enter to continue');
    pause(); 
    disp('UR5 Max Reach X axis');
    for i =0:pi/10:pi/2
        self.model.plot([0, i, i, 0, 0, 0, 0], 'workspace', self.workspace);
    end
    self.q =[0, pi/2, pi/2, 0, 0, 0, 0];
    maxReachKineX = self.model.fkine(self.q);
    maxReachUR5X = ([abs(maxReachKineX(1,4) - baseUR5(1,4)); 
                     abs(maxReachKineX(2,4) - baseUR5(2,4)); 
                     abs(maxReachKineX(3,4) - baseUR5(3,4))]);

    self.maxReachUR5X = maxReachUR5X;

    disp('Please press Enter to continue');
    pause(); 
    disp('UR5 Max Reach Y axis');
    for i =(pi/2):pi/10:pi
        self.model.plot([0, i, pi/2, 0, 0, 0, 0], 'workspace', self.workspace);
    end
    self.q =[0, pi, pi/2, 0, 0, 0, 0];
    maxReachKineY = self.model.fkine(self.q);
    maxReachUR5Y = ([abs(maxReachKineY(1,4) - baseUR5(1,4)); 
                     abs(maxReachKineY(2,4) - baseUR5(2,4)); 
                     abs(maxReachKineY(3,4) - baseUR5(3,4))]);
    
    self.maxReachUR5Y = maxReachUR5Y;

    disp('Please press Enter to continue');
    pause(); 
    disp('UR5 Max Reach Z axis');
    for i =pi/2:-pi/10:0
        self.model.plot([0, pi, i, 0, 0, 0, 0], 'workspace', self.workspace);
    end
    self.q =[0, 0, 0, 0, 0, 0, 0];
    maxReachKineZ = self.model.fkine(self.q);
    maxReachUR5Z = ([abs(maxReachKineZ(1,4) - baseUR5(1,4)); 
                     abs(maxReachKineZ(2,4) - baseUR5(2,4)); 
                     abs(maxReachKineZ(3,4) - baseUR5(3,4))]);

    self.maxReachUR5Z = maxReachUR5Z;

    %self.model.plot(self.qHome);

    disp ('Max Arm Reach from UR5 Base (x,y,z) = ')
    display(self.maxReachUR5X(1)), display(self.maxReachUR5Y(2)), display(self.maxReachUR5Z(3))
end

%%
function UR5ReachVolume(self)
    % Cycle through models Joint limits
    self.qlim = self.model.qlim();
    counter = 1;
    for q1 = self.qlim(1,1):(pi/10):self.qlim(1,2)
        for q2 = self.qlim(2,1):(pi/10):self.qlim(2,2)
            for q3 = self.qlim(3,1):(pi/10):self.qlim(3,2)
                endEff = self.model.fkine([q1, q2, q3, 0, 0, 0, 0]); % Point cloud
                self.endEffectorPoses(counter,:) = endEff(1:3,4)';
                counter = counter + 1;
            end
        end
    end
    % Plot poses of end effectors
    % https://au.mathworks.com/help/matlab/ref/plot3.html
    volume = plot3(self.endEffectorPoses(:,1), ...
                self.endEffectorPoses(:,2), ...
                self.endEffectorPoses(:,3));
                
    % https://au.mathworks.com/help/matlab/ref/convhull.html#mw_f9cda693-9c7a-40a0-84e0-7cee7b2af88b
    [~, self.volumeUR5] = convhull(self.endEffectorPoses(:,1), ...
                                   self.endEffectorPoses(:,2), ...
                                   self.endEffectorPoses(:,3));
    disp ('UR5 Arm Reach Volume = ')
    display (self.volumeUR5)
    disp('Please press Enter to continue');
    pause(); 

    set(volume, 'Visible', 'off');
end

    end
end