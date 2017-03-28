function [] = iDyntreeSimulator(t,chi,MODEL)
%IDYNTREESIMULATOR uses iDyntree visualizer to simulate robot movements.
%
% Format: [] = IDYNTREESIMULATOR(t,chi,MODEL)
%
% Inputs:  - current time t [s];
%          - state vector chi [13+4*ndof x 1];
%          - MODEL is a structure defining the robot model;
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% Setup environment and lights        
env = MODEL.VISUALIZER.viz.enviroment();        
env.setElementVisibility('root_frame',false);   
% set lights
sun = MODEL.VISUALIZER.viz.enviroment().lightViz('sun');         
sun.setDirection(MODEL.VISUALIZER.lightDir);
% set camera     
cam = MODEL.VISUALIZER.viz.camera();     
cam.setPosition(iDynTree.Position(MODEL.VISUALIZER.setPos(1),MODEL.VISUALIZER.setPos(2),MODEL.VISUALIZER.setPos(3)));     
cam.setTarget(iDynTree.Position(MODEL.VISUALIZER.setCamera(1),MODEL.VISUALIZER.setCamera(2),MODEL.VISUALIZER.setCamera(3)));           

%% Robot simulator
if length(t) == 1
    % this happens for online robot simulator
    [basePos,shape] = iDynTreeModelPositionFromChi(chi,MODEL);
    MODEL.VISUALIZER.viz.modelViz(MODEL.VISUALIZER.modelName).setPositions(basePos,shape);             
    % Draw visualizer       
    MODEL.VISUALIZER.viz.draw();
else
    counter     = 0; 
    fps         = 60;
    viz_ts      = t(1):(1/fps):t(end);
    createVideo = MODEL.CONFIG.makeVideo;
    % visualization loop 
    for ts = viz_ts
        tic        
        % find closest timestamp         
        [~,i] = min(abs(ts-t));                 
        % extract iDynTree model position from chi            
        [basePos,shape] = iDynTreeModelPositionFromChi(transpose(chi(i,:)),MODEL);               
        MODEL.VISUALIZER.viz.modelViz(MODEL.VISUALIZER.modelName).setPositions(basePos,shape);             
        % draw visualizer       
        MODEL.VISUALIZER.viz.draw(); 
        % record a video
        if createVideo 
            outputDir = './media';
            if (~exist(outputDir, 'dir'))
                mkdir(outputDir);
            end
            filename = strcat('./media/img',sprintf('%04d',counter),'.png');         
            MODEL.VISUALIZER.viz.drawToFile(filename);         
        end 
        % pause for the right amount of time        
        elapsedTime = toc;              
        % if we still need to wait       
        if( elapsedTime <= 1/fps )           
            pause(elapsedTime-1/fps);      
        end
        counter = counter+1;
    end
end

%% Video editing
if length(t) > 1
    if createVideo   
        videoName = 'videoSim';       
        fprintf('createVideo option enabled, imgXXXX.png created in the video directory\n');      
        fprintf('Matlab will now attempt to create a video running the following command in the video_comparison directory:\n');      
        command = strcat('ffmpeg -framerate 60 -i ./media/img%04d.png -c:v libx264 -r 30 -pix_fmt yuv420p ./media/',videoName,'.mp4'); 
        fprintf('%s\n',command);       
        system(command); 
        fprintf('Removing images...\n');      
        command = strcat('rm -Rf ./media/*.png'); 
        fprintf('%s\n',command);       
        system(command);  
       fprintf('Done!\n');  
   end
end
end