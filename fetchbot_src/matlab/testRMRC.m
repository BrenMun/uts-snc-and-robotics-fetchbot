clear; clc;
%% workspace
workspace = [-0.2 1.7 -1.2 1.2 0.2 1.5]; % workspace for matlab simulation
centerpnt = [1,0,0.25];  % Centre point for table proxy in matlab
robot = FetchRobot(transl(0,0,0.7) *trotz(pi), workspace);
environment = EnvironmentSetUp(workspace, 2.25, centerpnt);
axis equal; 
view(3);

%% set parameters
q = robot.model.getpos();
T1 = robot.model.fkine(q);
pointA = T1(1:3, end);
pointB = [0.6, 0.1, 0.75];

t = 2;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.5;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

%% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,8);       % Array for joint anglesR
qdot = zeros(steps,8);          % Array for joint velocities
xyzTraj = zeros(3,steps);       % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

%% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
d = 0.01;                            % desired distance from eef to point (in Z)
x0 = pointA(1);  x1 = pointB(1);
y0 = pointA(2);  y1 = pointB(2);
z0 = pointA(3) + d;  z1 = pointB(3) + d;
for i=1:steps
    xyzTraj(1,i) = (1-s(i))*x0 + s(i)*x1; % Points in x
    xyzTraj(2,i) = (1-s(i))*y0 + s(i)*y1; % Points in y
    xyzTraj(3,i) = (1-s(i))*z0 + s(i)*z1; % Points in z
end
theta0 = tr2rpy(T1);
% rotation means Z pointing down
T = transl(xyzTraj(1,1), xyzTraj(2,1), xyzTraj(3,1))...
    * trotx(theta0(1)) * troty(theta0(2)) * trotz(theta0(3));               % Create transformation of first point and angle
qMatrix(1,:) = q;                                                           % Solve joint angles to achieve first waypoint

%% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                                    % Get forward transformation at current joint state
    deltaX = xyzTraj(:,i+1) - T(1:3,4);                                     % Get position error from next waypoint
    Rd = rotx(pi);                                                          % Next rotation matrix is always the same (Z poiting down)
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(8))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:8
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = xyzTraj(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

for i = 1: length(qMatrix(:,1))
    robot.model.animate(qMatrix(i,:));
    drawnow();
    robot.steps = robot.steps + 1;
end 
