goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
endEffectorTr = blasterRobot.fkine(goalJointState);
% Ensure the Z component of the Z axis is positive (pointing upwards), and the Z component of the point is above 1 (approx mid height)

while endEffectorTr(3,3) < 0.1 || endEffectorTr(3,4) < 1
   goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
   endEffectorTr = blasterRobot.fkine(goalJointState);
   display('trying again');
end