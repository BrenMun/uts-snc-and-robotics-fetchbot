classdef Brick < handle
    %Bricks - This class creates objects of bricks and adds them into the
    
    properties
        % object name
        brick; 

        % Variables
        brickPose;
        v;

        % Brick states for State Machine
        state = 0; 
        unmoved = 0;
        targeted = 1; 
        moving = 3; 
        moved = 4; 
        unknown = 5;

    end
    
    methods
        function obj = Brick(x,y,z,i)
%             brick_h = PlaceObject('Brick.ply');
%             axis equal
%             vertices = get(brick_h,'Vertices');
%             
%             brickpose = ([x,y,z]);
% 
%             transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(brickpose)';
%             obj.model = set(brick_h,'Vertices',transformedVertices(:,1:3));  
% 
%             obj.brickPose = transl(brickpose); 

    %       from edit PLaceObjective, had to change to update positions
    %       with mesh
            [f,v,data] = plyread('Brick.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            brickpose = ([x,y,z]);
            obj.brickPose = transl(brickpose); %Input xyz
            obj.v = size(v,1); 
    
            locations = [obj.brickPose * [v,ones(size(v,1),1)]']'; 
            
            obj.brick = trisurf(f,locations(:,1), locations(:,2), locations(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        end

%         function RotateBrick(obj, Rads)
%             % Function to rotate brick
%             transformedVertices = [obj.v,ones(obj.v),1)] * transl(obj.brickpose) * troz(Rads)';
%             set(obj.brick,'Vertices',transformedVertices(:,1:3));  
%         end

        function updateBrickPose(obj, brickPose)
           
            locations = [obj.brickPose \ [obj.brick.Vertices,ones(obj.v,1)]']'; % inverted brick transform * verticies
            obj.brickPose = brickPose; % Update the brick pose to arm pose
            newLocations = [obj.brickPose * [locations(:,1:3),ones(obj.v,1)]']';
            obj.brick.Vertices = newLocations(:,1:3); 
        end
        
    end
end

