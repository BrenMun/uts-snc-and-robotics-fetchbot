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