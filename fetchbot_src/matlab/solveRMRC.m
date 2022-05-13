% RMRC based on robotics lab 9 
function [qMatrix, posError, angleError] = solveRMRC(robot, pointA, pointB, q0, totalTime, deltaT, distance)
    %% set up parameters
    numJoints = length(q0);                                                 % Number of joints
    t = totalTime;                                                          % Total time
    steps = t/deltaT;                                                       % Number of steps based on time
    epsilon = 0.5;                                                          % Manipulability threshold value
    W = diag([1 1 1 0.1 0.1 0.1]);                                          % Weighting matrix for velocity vector
    
    %% allocate array data
    m = zeros(steps,1);                                                     % measure of manipulability
    qMatrix = zeros(steps, numJoints);                                      % joint angles
    qdot = zeros(steps, numJoints);                                         % joint velocities
    pointTraj = zeros(3,steps);                                             % Point trajectory
    posError = zeros(3,steps);                                              % point trajectory error
    angleError = zeros(3,steps);                                            % joint trajectory error
    
    %% set up trajectory, initial pose
    s = lspb(0,1,steps);                                                    % trapezoidal trajectory
    distZ = distance;                                                       % desired distance from end-effector to point in Z
    x0 = pointA(1); y0 = pointA(2); z0 = pointA(3);                         % xyz of point1
    x1 = pointB(1); y1 = pointB(2); z1 = pointB(3) + distZ;                 % xyz of point2
    for i=1:steps
        pointTraj(1,i) = (1-s(i))*x0 + s(i)*x1;                             % points in x
        pointTraj(2,i) = (1-s(i))*y0 + s(i)*y1;                             % points in y
        pointTraj(3,i) = (1-s(i))*z0 + s(i)*z1;                             % points in z
    end        
    qMatrix(1,:) = q0;                                                       % solve joint angles for 1st waypoint

    %% track trajectory with RMRC
    for i=1:steps-1
        T = robot.fkine(qMatrix(i,:));                                      % fkine transform at current joints
        deltaPos = pointTraj(:,i+1) - T(1:3,4);                             % position error from next waypoint
        rotDesired = roty(pi);                                              % desired rotation matrix (z pointing down)
        rotActual = T(1:3,1:3);                                             % actual rotation matrix (current end-effector rotation matrix)
        rotError = (1/deltaT)*(rotDesired - rotActual);                     % rotation matrix error
        S = rotError * rotActual';                                          % skew symmetric matrix
        linear_velocity = (1/deltaT) * deltaPos;                            % linear velocity
        angular_velocity = [S(3,2); S(1,3); S(2,1)];                         % angular velocity
        rollPitchYaw = tr2rpy(rotDesired*rotActual);                        % roll-pitch-yaw of rotation matrix
        endEffectorVel = W * [linear_velocity;angular_velocity];              % end-effector velocity to reach next waypoint
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

