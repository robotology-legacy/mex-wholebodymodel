function SM = initStateMachine(CONFIG,state,mode)

ndof     = CONFIG.ndof;

%% CoM and position gains
gainPCOM = [10   50   10;   % state ==  1  TWO FEET BALANCING
            10   50   10;   % state ==  2  COM TRANSITION TO LEFT 
            10   50   10;   % state ==  3  LEFT FOOT BALANCING
            10   50   10;   % state ==  4  YOGA LEFT FOOT 
            10   50   10;   % state ==  5  PREPARING FOR SWITCHING 
            10   50   10;   % state ==  6  LOOKING FOR CONTACT
            10   50   10;   % state ==  7  TRANSITION TO INITIAL POSITION 
            10   50   10;   % state ==  8  COM TRANSITION TO RIGHT FOOT
            10   50   10;   % state ==  9  RIGHT FOOT BALANCING
            10   50   10;   % state == 10  YOGA RIGHT FOOT 
            10   50   10;   % state == 11  PREPARING FOR SWITCHING 
            10   50   10;   % state == 12  LOOKING FOR CONTACT
            10   50   10];  % state == 13  TRANSITION TO INITIAL POSITION


               % TORSO %%      LEFT ARM   %%     RIGHT ARM   %%         LEFT LEG       %%        RIGHT LEG      %% 
impedances  = [10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  1  TWO FEET BALANCING
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  2  COM TRANSITION TO LEFT 
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  3  LEFT FOOT BALANCING
               30  30  30,  10  10  10  10 10, 10  10  10  10 10, 50  50   100 100 50  50,  50  50  50  50  50  50   % state ==  4  YOGA LEFT FOOT 
               30  30  30,  10  10  10  10 10, 10  10  10  10 10, 30  50   100 60  50  50,  30  50  30  60  50  50   % state ==  5  PREPARING FOR SWITCHING 
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  6  LOOKING FOR CONTACT
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  7  TRANSITION TO INITIAL POSITION 
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  8  COM TRANSITION TO RIGHT FOOT
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  9  RIGHT FOOT BALANCING
               30  30  30,  10  10  10  10 10, 10  10  10  10 10, 50  50   50  50  50  50,  50  50  100 100 50  50   % state == 10  YOGA RIGHT FOOT 
               30  30  30,  10  10  10  10 10, 10  10  10  10 10, 30  50   30  60  50  50,  30  50  100 60  50  50   % state == 11  PREPARING FOR SWITCHING 
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50   % state == 12  LOOKING FOR CONTACT
               10  30  20,  10  10  10  8  8,  10  10  10  8  8,  30  50   30  60  50  50,  30  50  30  60  50  50]; % state == 13  TRANSITION TO INITIAL POSITION


SM.gainsVector = [gainPCOM(state,:),impedances(state,:)];

if state > 1
    
    SM.gainsVector_atPrevState = [gainPCOM(state-1,:),impedances(state-1,:)];
end
    
%% Joint references
qjReferences = [[ 0.0000, 0.0000,-0.1745, ...                              %% state == 1 TWO FEET BALANCING  
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000, ...      %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000];         %
                [-0.0348, 0.0779, 0.0429, ...                              %% state == 2  COM TRANSITION TO LEFT 
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.0015,-0.1109,-0.0001, 0.0003, 0.0160, 0.1630, ...      %  
                  0.0005, 0.0793,-0.0014,-0.0051, 0.0073,-0.1151];         %  
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 3  LEFT FOOT BALANCING
                  0.1253, 0.8135, 0.3051, 0.7928, 0.0000 ...               %   
                  0.0563, 0.6789, 0.3340, 0.6214, 0.0000 ...               %
                 -0.0015,-0.1109,-0.0001, 0.0003, 0.0160, 0.1630, ...      %  
                  0.0005, 0.0793,-0.0014,-0.0051, 0.0073,-0.1151];         %                        
                  zeros(1,ndof);                                           %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED         
                [-0.0348, 0.0779, 0.0429, ...                              %% state == 5  PREPARING FOR SWITCHING
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.0015,-0.1109,-0.0001, 0.0003, 0.0160, 0.1630, ...      %  
                  0.0005, 0.0793,-0.0014,-0.0051, 0.0073,-0.1151];         %                                  
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 6  LOOKING FOR CONTACT
                  0.1253, 0.8135, 0.3051, 0.7928, 0.0000 ...               %
                  0.0563, 0.6789, 0.3340, 0.6214, 0.0000 ...               %
                  0.0107,-0.0741,-0.0001,-0.0120, 0.0252, 0.1369,...       %
                 -0.0026, 0.0225, 0.0093,-0.0020, 0.0027,-0.0277];         %   
                  zeros(1,ndof);                                           %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 8  COM TRANSITION TO RIGHT FOOT
                  0.1253, 0.8135, 0.3051, 0.7928, 0.0000 ...               %
                  0.0563, 0.6789, 0.3340, 0.6214, 0.0000 ...               %
                  0.0107,-0.0741,-0.0001,-0.0120, 0.0252, 0.1369,...       %
                 -0.0026, 0.0225, 0.0093,-0.0020, 0.0027,-0.0277];         %  
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 9  RIGHT FOOT BALANCING
                  0.1253, 0.8135, 0.3051, 0.7928, 0.0000 ...               %    
                  0.0563, 0.6789, 0.3340, 0.6214, 0.0000 ...               %
                  0.0005, 0.0793,-0.0014,-0.0051, 0.0073,-0.1151, ...      %  
                 -0.0015,-0.1109,-0.0001, 0.0003, 0.0160, 0.1630];         %  
                  zeros(1,ndof);                                           %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                [-0.0348, 0.0779, 0.0429, ...                              %% state == 11  PREPARING FOR SWITCHING
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                  0.0005, 0.0793,-0.0014,-0.0051, 0.0073,-0.1151, ...      %  
                 -0.0015,-0.1109,-0.0001, 0.0003, 0.0160, 0.1630];         %                                  
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 12  LOOKING FOR CONTACT
                  0.1253, 0.8135, 0.3051, 0.7928, 0.0000 ...               %
                  0.0563, 0.6789, 0.3340, 0.6214, 0.0000 ...               %
                 -0.0026, 0.0225, 0.0093,-0.0020, 0.0027,-0.0277,...       %
                  0.0107,-0.0741,-0.0001,-0.0120, 0.0252, 0.1369];         %   
                  zeros(1,ndof);];                                         %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED 
              
SM.qjRef = qjReferences(state,:);

if state > 1
    
    SM.qjRef_atPrevState = qjReferences(state-1,:);
end

%% References for yoga ++
q1 = [-0.0790, 0.2279, 0.4519, ...
      -1.1621, 0.6663, 0.4919, 0.9947, 0.0000, ... 
      -1.0717, 1.2904,-0.2447, 1.0948, 0.0000, ...
       0.2092, 0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3484, 0.4008,-0.0004,-0.3672,-0.0530,-0.0875];
    
q2 = [-0.0790, 0.2279, 0.4519, ...
      -1.1621, 0.6663, 0.4965, 0.9947, 0.0000, ...
      -1.0717, 1.2904,-0.2493, 1.0948, 0.0000, ...
       0.2092, 0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ... 
       0.3714, 0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
   
   
q3 = [-0.0852,-0.4273, 0.0821,...
       0.1391, 1.4585, 0.2464, 0.3042, 0.0000, ...
      -0.4181, 1.6800, 0.7373, 0.3031, 0.0000, ...
       0.2092, 0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3714, 0.9599, 1.3253,-1.6594, 0.6374,-0.0614];  
   
q4 = [-0.0852,-0.4273, 0.0821,...
       0.1391, 1.4585, 0.2464, 0.3042, 0.0000, ...
      -0.4181, 1.6800, 0.7373, 0.3031, 0.0000, ...
       0.2092, 0.3473, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3514, 1.3107, 1.3253,-0.0189, 0.6374,-0.0614];
             
q5 = [-0.0790,-0.1273, 0.4519, ...
      -1.1621, 0.6663, 0.4965, 0.9947, 0.0000, ...
      -1.0717, 1.2904,-0.2493, 1.0948, 0.0000, ...
       0.2092, 0.3473, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3514, 1.3107, 1.3253,-0.0189, 0.6374,-0.0614];
             
             
q6 = [-0.0852,-0.4273, 0.0821,...
       0.1391, 1.4585, 0.2464, 0.3042, 0.0000, ...
      -0.4181, 1.6800, 0.7373, 0.3031, 0.0000, ...
       0.2092, 0.3473, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3514, 1.3107, 1.3253,-0.0189, 0.6374,-0.0614];  
             
q7 = [-0.0852,-0.4273, 0.0821,...
       0.1391, 1.4585, 0.2464, 0.3042, 0.0000, ...
      -0.4181, 1.6800, 0.7373, 0.3031, 0.0000, ...
       0.2092, 0.3473, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3514, 1.3107, 1.3253,-1.6217, 0.6374,-0.0614];
             
q8 = [-0.0852,-0.4273, 0.0821,...
       0.1391, 1.4585, 0.2464, 0.3042, 0.0000, ...
      -0.4181, 1.6800, 0.7373, 0.3031, 0.0000, ...
       0.2092, 0.3473, 0.0006,-0.1741,-0.1044, 0.0700, ...
       0.3514, 1.3107, 1.3253,-0.0189, 0.6374,-0.0614];            
             
if strcmp(mode,'1')

    SM.qjRef = q1;
    SM.qjRef_atPrevState = qjReferences(state-1,:);

elseif strcmp(mode,'2')
    
    SM.qjRef = q2;
    SM.qjRef_atPrevState = q1;

elseif strcmp(mode,'3')
              
    SM.qjRef = q3;
    SM.qjRef_atPrevState = q2;

elseif strcmp(mode,'4') 
    
    SM.qjRef = q4;
    SM.qjRef_atPrevState = q3;
          
elseif strcmp(mode,'5')          
          
    SM.qjRef = q5;
    SM.qjRef_atPrevState = q4;
     
elseif strcmp(mode,'6')     
          
    SM.qjRef = q6;
    SM.qjRef_atPrevState = q5;
     
elseif strcmp(mode,'7')       
          
    SM.qjRef = q7;
    SM.qjRef_atPrevState = q6;
      
elseif strcmp(mode,'8')    
          
    SM.qjRef = q8;
    SM.qjRef_atPrevState = q7;
end


%% CoM relative references
if strcmp(mode,'init') == 1
    
else
    
    xCoMRef = [(transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  1  TWO FEET BALANCING NOT USED
               (transpose((CONFIG.xCoMRef))+[0.0, 0.03, 0.0]);   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
               (transpose((CONFIG.xCoMRef))+[0.0, 0.02, 0.00]);  %% state ==  3  LEFT FOOT BALANCING 
               (transpose((CONFIG.xCoMRef))+[0.03,-0.01, 0.05]); %% state ==  4  YOGA LEFT FOOT
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  5  PREPARING FOR SWITCHING
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  6  LOOKING FOR CONTACT 
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
               % FROM NOW ON, THE REFERENCE ARE ALWAYS DELTAS W.R.T. THE POSITION OF THE RIGHT FOOT
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  8  COM TRANSITION TO RIGHT FOOT
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state ==  9  RIGHT FOOT BALANCING 
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state == 10  YOGA RIGHT FOOT
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state == 11  PREPARING FOR SWITCHING
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0]);   %% state == 12  LOOKING FOR CONTACT 
               (transpose((CONFIG.xCoMRef))+[0.0, 0.00, 0.0])];  %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
           
    SM.xCoMRef = xCoMRef(state,:);

    if state > 1
    
    SM.xCoMRef_atPrevState = xCoMRef(state-1,:);
    end
end

%% Smoothing times
SM.smoothingTimeJoints = 4;
SM.smoothingTimeGains  = 2;
SM.smoothingTimeCoM    = 2;
                 
% sm.joints.pointsR = sm.joints.pointsL;
% 
% 					 
% for i = 1:size(sm.joints.pointsR,1)				
% 	sm.joints.pointsR(i,2:4)          = [sm.joints.pointsR(i,2) -sm.joints.pointsR(i,3) -sm.joints.pointsR(i,4)];
% 	
% 	rightArm                           =  sm.joints.pointsR(i,end-15:end-12);
% 	sm.joints.pointsR(i,end-15:end-12) =  sm.joints.pointsR(i,end-19:end-16);
% 	sm.joints.pointsR(i,end-19:end-16) =  rightArm;
% 	
% 	rightLeg                          =  sm.joints.pointsR(i,end-5:end);
% 	sm.joints.pointsR(i,end-5:end)    =  sm.joints.pointsR(i,end-11:end-6);
% 	sm.joints.pointsR(i,end-11:end-6) =  rightLeg;
% end	 
% 
% clear q1 q2 q3 q4;
end
