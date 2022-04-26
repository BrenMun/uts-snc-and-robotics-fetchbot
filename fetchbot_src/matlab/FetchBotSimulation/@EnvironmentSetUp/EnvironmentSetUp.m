classdef EnvironmentSetUp < handle
    %ENVSETUP Summary of this class goes here
    %   Detailed explanation goes here
    
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


    end
    
    methods
        function obj = EnvironmentSetUp(workspace, tableClearence)
            %ENVSETUP Construct an instance of this class
            %   Detailed explanation goes here
            
            % Adding the Table
            obj.table = Table(-0.857, 0, 0);
           
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