function SM = initStateMachine(INIT_CONDITIONS,state,mode)
%INITSTATEMACHINE configures robot gains and references for each state of
%                 the finite state machine.
%
% SM = INITSTATEMACHINE(INIT_CONDITIONS,state,mode) takes as an input the 
% structure INIT_CONDITIONS, which contains robot initial conditions, the 
% current state and a 'mode' parameter. The output is the structure SM 
% containing state machine initialization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, February 2017
%

% ------------Initialization----------------

% number of DoFs
ndof     = INIT_CONDITIONS.CONFIG.ndof;

%% CoM and joint position gains
gainPCOM = [30   45   30;   % state ==  1  TWO FEET BALANCING
            30   45   30;   % state ==  2  COM TRANSITION TO LEFT 
            30   45   30;   % state ==  3  LEFT FOOT BALANCING
            30   45   30;   % state ==  4  YOGA LEFT FOOT 
            30   45   30;   % state ==  5  PREPARING FOR SWITCHING 
            30   45   30;   % state ==  6  LOOKING FOR CONTACT
            30   45   30;   % state ==  7  TRANSITION TO INITIAL POSITION 
            30   45   30;   % state ==  8  COM TRANSITION TO RIGHT FOOT
            30   45   30;   % state ==  9  RIGHT FOOT BALANCING
            30   45   30;   % state == 10  YOGA RIGHT FOOT 
            30   45   30;   % state == 11  PREPARING FOR SWITCHING 
            30   45   30;   % state == 12  LOOKING FOR CONTACT
            30   45   30];  % state == 13  TRANSITION TO INITIAL POSITION

               % TORSO %%      LEFT ARM   %%     RIGHT ARM   %%         LEFT LEG       %%        RIGHT LEG      %% 
impedances  = [20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  1  TWO FEET BALANCING
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  2  COM TRANSITION TO LEFT 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  3  LEFT FOOT BALANCING
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  30  50  50,  30  50  30  60  50  50   % state ==  4  YOGA LEFT FOOT 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  5  PREPARING FOR SWITCHING 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  6  LOOKING FOR CONTACT
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  7  TRANSITION TO INITIAL POSITION 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  8  COM TRANSITION TO RIGHT FOOT
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state ==  9  RIGHT FOOT BALANCING
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state == 10  YOGA RIGHT FOOT 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state == 11  PREPARING FOR SWITCHING 
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50   % state == 12  LOOKING FOR CONTACT
               20  30  20,  10  10  10  10  10,  10  10  10  10  10,  30  50   30  60  50  50,  30  50  30  60  50  50]; % state == 13  TRANSITION TO INITIAL POSITION

% select gains for the current state
SM.gainsVector = [gainPCOM(state,:),impedances(state,:)]/5;
    
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
                [ 0.0000, 0.0000,-0.1745, ...                              %% state == 7  TRANSITION INIT POSITION 
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000, ...      %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000];         %
                [ 0.0864, 0.0258, 0.0152, ...                              %% state == 8  COM TRANSITION TO RIGHT FOOT
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
                 -0.1493, 0.8580, 0.2437, 0.8710, 0.0000 ...               %
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
                [ 0.0000, 0.0000,-0.1745, ...                            %% state == 13  BALANCING TWO FEET   
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000, ...      %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000];];       %                                       
     
% current state reference
SM.qjRef = transpose(qjReferences(state,:));

%% References for yoga movements (state 4,state 10)
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
       
% update the current reference during state 4-10 using 'mode' variable
if strcmp(mode,'1')
    
    SM.qjRef = transpose(q1);

elseif strcmp(mode,'2')
    
    SM.qjRef = transpose(q2);

elseif strcmp(mode,'3')
              
    SM.qjRef = transpose(q3);

elseif strcmp(mode,'4') 
    
    SM.qjRef = transpose(q4);
          
elseif strcmp(mode,'5')          
          
    SM.qjRef = transpose(q5);
   
elseif strcmp(mode,'6')     
          
    SM.qjRef = transpose(q6);
     
elseif strcmp(mode,'7')       
          
    SM.qjRef = transpose(q7);
      
elseif strcmp(mode,'8')    
          
    SM.qjRef = transpose(q8);
end

%% CoM reference + a configurable delta 
if strcmp(mode,'init') == 1
    % in this case, CoM references are given by initForwardKinematics
else
    xCoMRefDelta = [0.0, 0.0, 0.0;   %% state ==  1  TWO FEET BALANCING 
                    0.0, 0.0, 0.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                    0.0, 0.0, 0.0;   %% state ==  3  LEFT FOOT BALANCING 
                    0.0, 0.0, 0.1;   %% state ==  4  YOGA LEFT FOOT
                    0.0, 0.0, 0.0;   %% state ==  5  PREPARING FOR SWITCHING
                    0.0, 0.0, 0.0;   %% state ==  6  LOOKING FOR CONTACT 
                    0.0, 0.0, 0.0;   %% state ==  7  TRANSITION INIT POSITION
                    0.0, 0.0, 0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                    0.0, 0.0, 0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                    0.0, 0.0, 0.0;   %% state == 10  YOGA RIGHT FOOT
                    0.0, 0.0, 0.0;   %% state == 11  PREPARING FOR SWITCHING
                    0.0, 0.0, 0.0;   %% state == 12  LOOKING FOR CONTACT 
                    0.0, 0.0, 0.0];  %% state == 13  TRANSITION INIT POSITION
       
    % set current CoM reference
    SM.xCoMRef = INIT_CONDITIONS.xCoMRef + transpose(xCoMRefDelta(state,:));
end

%% Tresholds and smoothing times
SM.TRESHOLDS.t_treshold     = [1 1 2         0 2 1   4 1.5  1    2 2 2 5];
SM.TRESHOLDS.f_treshold     = [0 180 0       0 0 0   0 110  0    0 0 0 0];
SM.TRESHOLDS.com_treshold   = [0 0.01 0.005  0 0 0   0 0.05 0    0 0 0 0];

SM.smoothingTimeJoints      = 0.5;
SM.smoothingTimeGains       = 0.5;
SM.smoothingTimeCoM         = 0.5;
SM.t_yoga                   = 3;
       
%% Invert references for right foot balancing and Yoga
if state  == 10
    % torso				 
	SM.qjRef(1:3)   = [SM.qjRef(1) -SM.qjRef(2) -SM.qjRef(3)];		
	% arms
	rightArm        =  SM.qjRef(9:13);
	SM.qjRef(9:13)  =  SM.qjRef(4:8);
	SM.qjRef(4:8)   =  rightArm;	
	% legs
	rightLeg        =  SM.qjRef(20:25);
	SM.qjRef(20:25) =  SM.qjRef(14:19);
	SM.qjRef(14:19) =  rightLeg;        
end
end
