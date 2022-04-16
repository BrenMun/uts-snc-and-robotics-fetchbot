classdef Cone < handle
    %CONE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;  
       
    end
    
    methods
        function obj = Cone(x1,y1,x2,y2,height,tableLength)
            %CONE Construct an instance of this class
            %   4 cones will be placed on the ground around the robots, 
            % find a midpoint between robots, by enetering xyz of the 2
            % bots then place them a determined distance away. Use the
            % table height to find ground. 

            P1 = [x1, y1];
            P2 = [x2, y2];
            middleP = (P1(:) + P2(:)).'/2 ;
            x=middleP(1);
            y=middleP(2);

            %1
            cone_h = PlaceObject('cone.ply');
            axis equal
            vertices = get(cone_h,'Vertices');

            conepose = ([x+tableLength, y+tableLength, -height]);

            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(conepose)' * troty(0)';
            set(cone_h,'Vertices',transformedVertices(:,1:3));
            
            %2
            cone_h = PlaceObject('cone.ply');
            axis equal
            vertices = get(cone_h,'Vertices');

            conepose = ([x+tableLength, y-tableLength, -height]);
            
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(conepose)' * trotz(0)';
            set(cone_h,'Vertices',transformedVertices(:,1:3));
            
            %3
            cone_h = PlaceObject('cone.ply');
            axis equal
            vertices = get(cone_h,'Vertices');

            conepose = ([x-tableLength, y-tableLength, -height]);
            
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(conepose)' * trotx(0)';
            set(cone_h,'Vertices',transformedVertices(:,1:3));
            
            %4
            cone_h = PlaceObject('cone.ply');
            axis equal
            vertices = get(cone_h,'Vertices');

            conepose = ([x-tableLength, y+tableLength, -height]);
            
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(conepose)' * trotx(0)';
            set(cone_h,'Vertices',transformedVertices(:,1:3));
        end
        
    end
end

