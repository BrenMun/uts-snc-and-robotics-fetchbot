classdef EmergencyStop < handle
    %EMERGENCYSTOP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = EmergencyStop(tableWidth, tableLength)
            %EMERGENCYSTOP Construct an instance of this class
            %   Detailed explanation goes here
            estop = PlaceObject('EStop.ply');
            axis equal
            vertices = get(estop,'Vertices');

            eStopPose = [tableWidth, tableLength, 0.05];
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(eStopPose)';
            set(estop,'Vertices',transformedVertices(:,1:3));
        end
        
    end
end

