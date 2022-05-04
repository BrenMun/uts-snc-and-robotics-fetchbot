%% Working out UR3 Max Reach
clf 
clc 
clear all

L1 = Link('d',0.1519 ,'a',0       ,'alpha', pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d',0      ,'a',-0.24365,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L3 = Link('d',0      ,'a',-0.21325,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L4 = Link('d',0.11235,'a',0       ,'alpha', pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L5 = Link('d',0.08535,'a',0       ,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L6 = Link('d',0.0819 ,'a',0       ,'alpha', 0   ,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);   

% UR3 DH parameters found at: 
% https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

UR3 = SerialLink([L1 L2 L3 L4 L5 L6]);


UR3x = 0;
UR3y = 0;
UR3z = 0;
UR3.base = transl(UR3x,UR3y,UR3z);
                                
workspace = [-0.5 0.5 -0.5 0.5 -1 1]; 
scale = 0.2;        
q = zeros(1,6);                                                           
UR3.plot(q,'workspace',workspace,'scale',scale);                  
hold on

%
UR3.teach

%%
% Calculations for max reach

for i =0:-pi/10:-pi/2
    UR3.plot([0, 0, 0, i, 0, 0]);
end
%deg2rad(UR3.getpos)
reach = UR3.fkine(deg2rad(UR3.getpos));
maxReach = reach(1:3,4);
maxReachX = [reach(1,4)-UR3.base(1), reach(2,4)-UR3.base(2), reach(3,4)-UR3.base(3)]

for i =0:-pi/10:-pi/2
    UR3.plot([i, 0, 0, -pi/2, 0, 0]);
end
reach = UR3.fkine(deg2rad(UR3.getpos));
maxReach = reach(1:3,4);
maxReachY = [reach(1,4)-UR3.base(1), reach(2,4)-UR3.base(2), reach(3,4)-UR3.base(3)]

for i =0:-pi/10:-pi/2
    UR3.plot([-pi/2, i, 0, -pi/2, 0, 0])
end
reach = UR3.fkine(deg2rad(UR3.getpos));
maxReach = reach(1:3,4);
maxReachZ = [reach(1,4)-UR3.base(1), reach(2,4)-UR3.base(2), reach(3,4)-UR3.base(3)]


%% Working out UR5 Max Reach
clf 
clc 
clear all

    %L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
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
% UR3 DH parameters found at: 
% https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

UR5 = SerialLink([L(2) L(3) L(4) L(5) L(6) L(7)]);

UR5x = 0;
UR5y = 0;
UR5z = 0;
UR5.base = transl(UR5x,UR5y,UR5z);
                                
workspace = [-1.5 1.5 -1.5 1.5 -1.5 1.5]; 
scale = 0.2;        
q = zeros(1,6);                                                           
UR5.plot(q,'workspace',workspace,'scale',scale);                  
hold on

UR5.teach

%%

x = zeros(3,3)

x(:,1)


%% Fetch Robot
clf 
clc 
clear all

% https://www.google.com/url?sa=i&url=http%3A%2F%2Fragazzini.me.vt.edu%2Fpdf%2FLearning_IK_ID_GAN_RAS_2020.pdf&psig=AOvVaw1Rba0hDu8iqFIiWcYX9SmQ&ust=1650853547949000&source=images&cd=vfe&ved=0CA0QjhxqFwoTCPC9kO7Sq_cCFQAAAAAdAAAAABAm

L0 = 0.110;  % x
D1 = 0.0;  % 
L1 = 0.1197;  % 

L2 = 0.1163;  % Shoulder pan joint
L3 = 0.17181;  % Shoulder lift joint
L4 = 0.18093;  % Upper arm roll joint
L5 = 0.1658;  % elbow flex joint
L6 = 0.15675;  % forarm roll joint
L7 = 0.1166;  % wrist flex joint
L8 = 0.04224;  % wrist roll joint
L9 = 0.12193;  % gripper joint

    L(1) = Link([pi     L1+D1   L0         0      1]); % PRISMATIC Link shulder pan
    L(2) = Link([0      L3      L2        -pi/2   0]); % SHoulder lift
    L(3) = Link([0      0       0         -pi/2   0]); % upper arm rol joint
    L(4) = Link([0      L4+L5   0          pi/2   0]);
    L(5) = Link([0      0       0         -pi/2   0]);
    L(6) = Link([0      L6+L7   0          pi/2	  0]);
    L(7) = Link([0      0       0         -pi/2   0]);
    L(8) = Link([0      L8+L9   0          0      0]);
    %            ?      d       x          Alpa   type
    
    % Incorporate joint limits
    L(1).qlim = [0 0.400];
    L(2).qlim = [-23/45*pi, 23/45*pi];
    L(3).qlim = [-7/18*pi, 29/60*pi];
    L(4).qlim = [-360 360]*pi/180; %continuous not sure how to do that
    L(5).qlim = [-43/60*pi, 43/60*pi];
    L(6).qlim = [-360 360]*pi/180; %continuous
    L(7).qlim = [-25/36*pi, 25/36*pi ];
    L(8).qlim = [-360 360]*pi/180; %continuous

    L(3).offset = -pi/2;
    L(5).offset = -pi/2;
    
    fetchBot = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8)]);
    fetchBot.base = transl(0,0,0);
                                    
    workspace = [-1.5 1.5 -1.5 1.5 -1.5 1.5]; 
    scale = 0.2;        
    q = zeros(1,8);                                                           
    fetchBot.plot(q,'workspace',workspace,'scale',scale);                  
    hold on
    
    fetchBot.teach
    %Add table like you added the cube in lab 5
    % add is collision and avoid collison in
    % change from 3 dof to 8? dof
    % Set up way points for bin movement
    % 

 %% https://au.mathworks.com/help/robotics/ug/check-for-manipulator-self-collisions-using-collision-meshes.html
clear 
clc
clf 
iiwa = importrobot('fetch.urdf');
iiwa.DataFormat = 'column';

startConfig = [0 -pi/4 pi 3*pi/2 0 -pi/2 pi/8]';
goalConfig = [0 -pi/4 pi 3*pi/4 0 -pi/2 pi/8]';

q = trapveltraj([startConfig goalConfig],100,'EndTime',3);


% isConfigInCollision = false(100,1);
% configCollisionPairs = cell(100,1);
% sepDistForConfig = zeros(iiwa.NumBodies+1,iiwa.NumBodies+1,100);
% for i = 1:length(q)
%     [isColliding, sepDist] = checkCollision(iiwa,q(:,i),'Exhaustive','on');
%     isConfigInCollision(i) = isColliding;
%     sepDistForConfig(:,:,i) = sepDist;
% end
figure;
show(iiwa)
%show(iiwa,q(:,firstCollisionIdx));
%exampleHelperHighlightCollisionBodies(iiwa,configCollisionPairs{firstCollisionIdx}+1,gca);

%% 