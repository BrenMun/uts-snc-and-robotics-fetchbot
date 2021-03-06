%%
% This script is for quick testing and fiddling. 
%% main testing
%% Teach for gui control
simulation.teaching; 
%% Get arms current position
simulation.getPos()
%% Test movement
%qObject = [0    0.8219   -0.0780   -0.8516   -1.2403    1.0248   -1.4547    2.6003];
%qObject = [0    0.9346    0.0698    1.5080   -0.3339    0.1255   -2.0972    1.2579];
qObject = [0    1.4806   -0.0398    0.0000   -0.5591   -0.0001    1.5680    0.0012];
simulation.MoveArm(qObject);
%% Move to bin
qObject =[0.0640    1.3200   -0.1495   -0.1256   -0.8292   -0.1258    0.8263    0.0012];
simulation.MoveArm(qObject);
%% check current pos for collisions with table
simulation.checkCollisions(simulation.robotFetch);
%% simulation reset
qObject = simulation.robotFetch.qHome;
simulation.MoveArm(qObject);
%%
simulation.getFkine();





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

%%
% Add starting Poses for Fetch Robot
k = transl(0,0,0.7) 

l = transl(0,0,0.7) *trotz(pi)



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

%% testing collision detetion with fetch bot
clc
clear all
clf 

fetchBase = transl(1,0,0) *trotz(pi);
centerpnt = [2,0,-0.5];

workspace = [-1 3 -1 1 -0.5 1.5];

robotFetch = FetchRobot(fetchBase,workspace);
q = robotFetch.qHome;

side = 1;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
camlight




%%
robotFetch.teaching;
%%
L = robotFetch.model.links; % gets robot links
tr = zeros(4,4,length(L) + 1+1); % initialised transfor as 4 by 4, and number of joints plus one
tr(:,:,1) = fetchBase;

for i = 1 : length(L)
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end



%%
q1 = q;
q2 = [0    0.1639    1.4362   -0.1256   -1.7749   -0.0001   -0.5700    0.0012];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end % this while loop, loops until it finds enough steps less and 1 degree
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robotFetch,qMatrix(i,:),faces,vertex,faceNormals,false);
    robotFetch.model.animate(qMatrix(i,:));
      if result == 1
        disp(['Intersection at step ', num2str(i)]);
        break;
    end
end
%% lab 5 directly

clf 
clc 
clear
% https://www.youtube.com/watch?v=CXw1udCyBvE&ab_channel=mewgen
% https://www.youtube.com/watch?v=tce28zET9kI&ab_channel=mewgen

% 2.1) Create a 3 link planar robot with all 3 links having a = 1m, leave the base at eye(4).
%fetchBase = transl(0.5,-1,0) *trotz(pi);
workspace = [-1 3 -1.5 1.5 -1.5 1.5];
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


robot = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8)],'name','myRobot');    
robot.base = transl(1,-1,0) *trotz(pi);

%robotFet = FetchRobot(fetchBase,workspace);
%robot = robotFet.model;
q = zeros(1,8);                                                     % Create a vector of initial joint angles        
scale = 0.5;
%workspace = [-2 2 -2 2 -0.05 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot


% 2.2) Put a cube with sides 1.5m in the environment that is centered at [2,0,-0.5].
% 2.3) Use teach and note when the links of the robot can collide with 4 of the planes:

centerpnt = [2,0,-0.5];
%centerpnt = [0,-1.7,-0.5];
side = 1.5;
plotOptions.plotFaces = true;
plotOptions.plotVerts = true;
plotOptions.plotEdges = true;

[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
camlight



% Get the transform of every joint (i.e. start and end of every link)
L = robot.links(end-6:end); % gets robot links
tr = zeros(4,4,length(L)+1); % initialised transfor as 4 by 4, and number of joints plus one

%tr(:,:,1) = robot.base*trotz(pi); %%%%%%% do i need to rotate again???
tr(:,:,1) = robot.base;
points = [];
xypoint=[];
xyzpoint=[];

for i = 1 : length(L)
    tr(:,:,i+1) = tr(:,:,i)*trotz(pi) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);

    hold on
    plot3(tr(1,4,i),tr(2,4,i),tr(3,4,i));
    axis equal
    view(3)
    points = [points, tr(1,4,i)];% tr(2,4,i) tr(3,4,i)];
    xypoint = [xypoint,tr(2,4,i)];
    xyzpoint =[xyzpoint,tr(3,4,i)];
end
points
xypoint
xyzpoint
%plot(xypoint);
plot3(points,xypoint,xyzpoint);

% hold on
% for i = 1:length(points)
%     hold on
%     plot3(i, i+1, i+2); 
%     axis equal
% end

% 2.5: Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
        

    end    
end
%%

%  Go through until there are no step sizes larger than 1 degree using diff(rad2deg(jtraj(q1,q2,steps))
q1 = [0,0,0,0,0,0,0,0];
q2 = [0, pi/4,0,0,0,0,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end % this while loop, loops until it finds enough steps less and 1 degree
qMatrix = jtraj(q1,q2,steps);
disp("Steps ")
steps
%

% 2.7)Check each of the joint states in the trajectory to work out which 
% ones are in collision. Return a logical vector of size steps which 
% contains 0 = no collision (safe) and 1 = yes collision (Unsafe). 
% You may like to use this structure.
result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end

% This finds the joint states that are in collision
% inside is collision, 


%% testing lab 6 stuff

clc
clear all
clf 

fetchBase = transl(0,0,0) *trotz(pi);
centerpnt = [2,0,-0.5];

workspace = [-1 3 -1 1 -0.5 1.5];

robotFetch = FetchRobot(fetchBase,workspace);
q = robotFetch.qHome;

% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)]; 




%%
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            intersectP;
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
            
        end    
    end
end
end
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links(end-6:end);
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);

    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
















