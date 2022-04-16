classdef Table < handle
    %TABLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       
    end
    
    methods
        function obj = Table(x, y, z)
            %TABLE Construct an instance of this class
            %   Detailed explanation goes here
            table_h = PlaceObject('Welded_Table.ply');
            axis equal
            vertices = get(table_h,'Vertices');
            
            %tablepose = ([-1.75 0 0]);
            tablepose = ([x, y, z]);
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(tablepose)'* trotx(pi/2)';
            set(table_h,'Vertices',transformedVertices(:,1:3));
        end
    end
end

