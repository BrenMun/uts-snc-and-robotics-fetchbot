classdef Table < handle
    %TABLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       tableVertices;
       tableFace;
       tableFaceNormals;

    end
    
    methods
        function obj = Table(x, y, z)
            %TABLE Construct an instance of this class
            %   Detailed explanation goes here
            table_h = PlaceObject('Welded_Table.ply');
            axis equal
            vertices = get(table_h,'Vertices');
            obj.tableVertices = vertices; 
            
            %tablepose = ([-1.75 0 0]);
            tablepose = ([x, y, z]);
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(tablepose)'* trotx(pi/2)';
            set(table_h,'Vertices',transformedVertices(:,1:3));

            obj.tableFace=[1,2,3;1,3,7;
                         1,6,5;1,7,5;
                         1,6,4;1,4,2;
                         6,4,8;6,5,8;
                         2,4,8;2,3,8;
                         3,7,5;3,8,5;
                         6,5,8;6,4,8];
        
        %if 2 < nargout    
        obj.tableFaceNormals = zeros(size(face,1),3);
            for faceIndex = 1:size(face,1)
                v1 = vertex(face(faceIndex,1)',:);
                v2 = vertex(face(faceIndex,2)',:);
                v3 = vertex(face(faceIndex,3)',:);
                faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
        end
    end
end

