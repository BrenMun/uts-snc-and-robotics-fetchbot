% RMRC based on robotics lab 9 
function [qMatrix, posError, angleError] = solveRMRC(robot, point1, point2, q0, totalTime, deltaT)
    %% set up parameters
    numJoints = length(q0);                                                 % Number of joints
    t = totalTime;                                                          % Total time
    steps = t/deltaT;                                                       % Number of steps based on time
    epsilon = 0.1;                                                          % Manipulability threshold value
    W = eye(numJoints);                                                     % Weighting matrix for velocity vector
    
    %% allocate array data
    m = zeros(steps,1);                                                     % measure of manipulability
    qMatrix = zeros(steps, numJoints);                                      % joint angles
    qdot = zeros(steps, numJoints);                                         % joint velocities
    pointTraj = zeros(3,steps);                                             % Point trajectory
    posError = zeros(3,steps);                                              % point trajectory error
    angleError = zeros(3,steps);                                            % joint trajectory error
    
    %% set up trajectory, initial pose
    s = lspb(0,1,steps);                                                    % trapezoidal trajectory
    distZ = 0.3;                                                            % desired distance from end-effector to point in Z
    x1 = point1(1); y1 = point1(2); z1 = point1(3) + distZ;                 % xyz of point1
    x2 = point2(1); y2 = point2(2); z2 = point2(3) + distZ;                 % xyz of point2

    for i=1:steps
        pointTraj(1,i) = (1-s(i))*x1 + s(i)*x2;                             % points in x
        pointTraj(2,i) = (1-s(i))*y1 + s(i)*y2;                             % points in y
        pointTraj(3,i) = (1-s(i))*z1 + s(i)*z2;                             % points in z
    end    
    
    T = transl(pointTraj(1,1), pointTraj(2,1), pointTraj(3,1)) * trotx(pi); % transformation matrix with z pointing down
    qMatrix(1,:) = robot.ikcon(T,q0);                                       % solve joint angles for 1st waypoint
    robot.plot(qMatrix(1,:));                                               % plot initial joints

    %% track trajectory with RMRC
    for i=1:steps-1
        T = robot.fkine(qMatrix(i,:));                                      % fkine transform at current joints
        deltaPos = pointTraj(:,i+1) - T(1:3,4);                             % position error from next waypoint
        rotDesired = rotx(pi);                                              % desired rotation matrix (z pointing down)
        rotActual = T(1:3,1:3);                                             % actual rotation matrix (current end-effector rotation matrix)
        rotError = (1/deltaT)*(rotDesired - rotActual);                     % rotation matrix error
        S = rotError * rotActual';                                          % skew symmetric matrix
        linearVelocity = deltaPos / deltaT;                                 % linear velocity
        angularVelocity = [S(3,2); S(1,3); S(2,1)];                         % angular velocity
        rollPitchYaw = tr2rpy(rotDesired*rotActual);                        % roll-pitch-yaw of rotation matrix
        endEffectorVel = W*[linear_velocity;angular_velocity];              % end-effector velocity to reach next waypoint
        J = robot.jacob0(qMatrix(i,:));                                     % jacobian at current joint state
        m(i) = sqrt(det(J*J'));                                             % calculate manipulability

        % set gain if manipulability is less than threshold
        if m(i) < epsilon
            lambda = (1 - m(i)/epsilon)*5E-2;                               % insufficient manipulability, damping needed
        else
            lambda = 0;                                                     % sufficient manipulability, no damping needed
        end
        
        invJ = inv(J'*J + lambda * eye(numJoints)) * J';                    % damped least squares inverse
        qdot(i,:) = (invJ*endEffectorVel)';                                 % solve RMRC equation
        
        for j=1:numJoints
            if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)             % If next joint angle is lower than joint limit
                qdot(i,j) = 0;                                              %   Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)         % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0;                                              %   Stop the motor
            end   
        end

        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                   % Update next joint state based on joint velocities
        posError(:,i) = pointTraj(:,i+1) - T(1:3,4);                        % position error
        angleError(:,i) = rollPitchYaw;                                     % angle error
    end
end

