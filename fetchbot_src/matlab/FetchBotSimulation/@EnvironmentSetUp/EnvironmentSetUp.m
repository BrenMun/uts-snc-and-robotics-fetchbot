classdef EnvironmentSetUp < handle
    %ENVSETUP Sets up the environemnt for matlab
    %   This class creates objects of classes that make up the environment
    %   of the matlab simulation
    
    properties
    model; 
    tableTolerance = 0.5;
    w1;
    w2;
    w3;
    table;
    cone;
    fireExtinguisher;
    eStop;
    
    tableVertices;
    tableFaces;
    tableFaceNormals;
    


    end
    
    methods
        function obj = EnvironmentSetUp(workspace, tableClearence, centerpnt)
            %ENVSETUP Construct an instance of this class
            %   Detailed explanation goes here
            
            % Adding the Table
            % obj.table = Table(-0.857, 0, 0);
            % centerpnt = [0,0.3,-0.7];
            side = 1;
            plotOptions.plotFaces = true;
            [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
            axis equal
            camlight
            obj.tableFaces = faces;
            obj.tableVertices = vertex; 
            obj.tableFaceNormals = faceNormals; 
           
           
            % Adding cones
            %obj.cone = Cone(0,0,0,0,1,tableClearence);
            
            % % Adding concrete
            %w1 = workspace(2); 
            %w2 = workspace(5); 
            %w3 = workspace(6);
            %surf([-w1,-w1;w1,w1],[-w1,w1;-w1,w1],[w2,w2;w2,w2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            % % Adding Wall
            %surf(-1*[w1,w1;w1,w1],-1*[-w1,w1;-w1,w1],[w2,w2;w3,w3],'CData',imread('wall.jpg'),'FaceColor','texturemap');
            %surf(-1*[-w1,w1;-w1,w1],-1*[w1,w1;w1,w1],[w2,w2;w3,w3],'CData',imread('wall.jpg'),'FaceColor','texturemap');
            
            % workspace = [-4 4 -4 4 -1.2 2.5];
            %wX = workspace(2)-1;
            %wY = workspace(3)+0.25;
            %wZ = workspace(5)*-1;
            % %Adding Fire estinguisher
            %obj.fireExtinguisher = FireExtinguisher(wX,wY,0.5);

            % %Adding EStop
            %obj.eStop = EmergencyStop(0.857, 1.65);

        end

        
        
    end
end

% %%
% % Adding the Table
% Table(-0.857, 0, 0);
% % Adding cones
% Cone(0,0,0,0,1,2.25);
% % Adding concrete
% w1 = workspace(2);w2 = workspace(5);w3 = workspace(6);
% surf([-w1,-w1;w1,w1],[-w1,w1;-w1,w1],[w2,w2;w2,w2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% % Adding Wall
% surf([w1,w1;w1,w1],[-w1,w1;-w1,w1],[w2,w2;w3,w3],'CData',imread('wall.jpg'),'FaceColor','texturemap');
% surf([-w1,w1;-w1,w1],[w1,w1;w1,w1],[w2,w2;w3,w3],'CData',imread('wall.jpg'),'FaceColor','texturemap');
