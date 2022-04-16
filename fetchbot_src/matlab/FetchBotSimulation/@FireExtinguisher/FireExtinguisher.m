classdef FireExtinguisher < handle
    %TABLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       
    end
    
    methods
        function obj = FireExtinguisher(x,y,z)
            %TABLE Construct an instance of this class
            %   Detailed explanation goes here
            fireE = PlaceObject('FireExtinguisher.ply');
            axis equal
            vertices = get(fireE,'Vertices');

            fireEPose = [x, y, z];
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(fireEPose)';
            set(fireE,'Vertices',transformedVertices(:,1:3));
        end
    end
end
