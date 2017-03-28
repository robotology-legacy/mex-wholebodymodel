function SM = initStateMachine(INIT_CONDITIONS, MODEL, state, prefix)
%INITSTATEMACHINE configures robot control gains, CoM and joint references and 
%                 tresholds for each state of the finite state machine.
%
% Format: SM = INITSTATEMACHINE(INIT_CONDITIONS, state, prefix)
%
% Inputs:  - INIT_CONDITIONS is a structure containing initial conditions
%            for integration;
%          - MODEL is a structure defining the robot model;
%          - state    a scalar that indicates the current state of the
%            finite state machine;
%          - prefix   if setted to 'number from 1 to 8' this function will 
%            provide references for yoga movements (only for state 4 and 
%            state 10).
%
% Output:  - SM it is a structure containing updated control gains,
%            references and tresholds for each state. 
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% CoM and joint position gains
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

% select proper gains for the current state
SM.gainsVector = [gainPCOM(state,:),impedances(state,:)]/10;
    
%% Joint references
% config parameters
ndof         = length(impedances(state,:));

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
                [ 0.0000, 0.0000,-0.1745, ...                              %% state == 13  BALANCING TWO FEET   
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                 -0.6278, 0.5231, 0.0010, 0.8727, 0.0000, ...              %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000, ...      %
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0244, 0.0000];];       %                                       
     
% joint reference for the current state
SM.qjRef = transpose(qjReferences(state,:));

%% References for yoga movements (state 4 and state 10)
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
       
% update the current reference during state 4-10 using 'prefix' variable
if strcmp(prefix,'1')    
    SM.qjRef = transpose(q1);
elseif strcmp(prefix,'2')    
    SM.qjRef = transpose(q2);
elseif strcmp(prefix,'3')              
    SM.qjRef = transpose(q3);
elseif strcmp(prefix,'4')    
    SM.qjRef = transpose(q4);          
elseif strcmp(prefix,'5')                   
    SM.qjRef = transpose(q5);  
elseif strcmp(prefix,'6')              
    SM.qjRef = transpose(q6);    
elseif strcmp(prefix,'7')                
    SM.qjRef = transpose(q7);      
elseif strcmp(prefix,'8')              
    SM.qjRef = transpose(q8);
end

%% CoM reference + a configurable delta 
if strcmp(prefix,'init') == 1
    % in this case, CoM references are given by initForwardKinematics
else
    xCoMRefDelta = [0.0, 0.0, 0.00;   %% state ==  1  TWO FEET BALANCING
                    0.0, 0.0, 0.00;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                    0.0, 0.0, 0.00;   %% state ==  3  LEFT FOOT BALANCING 
                    0.0, 0.0, 0.06;   %% state ==  4  YOGA LEFT FOOT
                    0.0, 0.0, 0.00;   %% state ==  5  PREPARING FOR SWITCHING
                    0.0, 0.0, 0.00;   %% state ==  6  LOOKING FOR CONTACT 
                    0.0, 0.0, 0.00;   %% state ==  7  TRANSITION INIT POSITION
                    0.0, 0.0, 0.00;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                    0.0, 0.0, 0.00;   %% state ==  9  RIGHT FOOT BALANCING 
                    0.0, 0.0, 0.06;   %% state == 10  YOGA RIGHT FOOT
                    0.0, 0.0, 0.00;   %% state == 11  PREPARING FOR SWITCHING
                    0.0, 0.0, 0.00;   %% state == 12  LOOKING FOR CONTACT 
                    0.0, 0.0, 0.00];  %% state == 13  TRANSITION INIT POSITION
       
    % set current CoM reference
    SM.xCoMRef = INIT_CONDITIONS.xCoMRef + transpose(xCoMRefDelta(state,:));
end

%% Tresholds and smoothing times
% time treshold
t_treshold = [0.8;   %% state ==  1  TWO FEET BALANCING 
              1.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT
              2.0;   %% state ==  3  LEFT FOOT BALANCING 
              0.0;   %% state ==  4  YOGA LEFT FOOT
              2.0;   %% state ==  5  PREPARING FOR SWITCHING
              1.0;   %% state ==  6  LOOKING FOR CONTACT 
              4.0;   %% state ==  7  TRANSITION INIT POSITION
              1.5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
              1.0;   %% state ==  9  RIGHT FOOT BALANCING 
              2.0;   %% state == 10  YOGA RIGHT FOOT
              2.0;   %% state == 11  PREPARING FOR SWITCHING
              2.0;   %% state == 12  LOOKING FOR CONTACT 
              5.0];  %% state == 13  TRANSITION INIT POSITION
          
% force treshold
f_treshold = [0.000;   %% state ==  1  TWO FEET BALANCING 
              145.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT
              0.000;   %% state ==  3  LEFT FOOT BALANCING 
              0.000;   %% state ==  4  YOGA LEFT FOOT
              0.000;   %% state ==  5  PREPARING FOR SWITCHING
              10.00;   %% state ==  6  LOOKING FOR CONTACT 
              0.000;   %% state ==  7  TRANSITION INIT POSITION
              145.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
              0.000;   %% state ==  9  RIGHT FOOT BALANCING 
              0.000;   %% state == 10  YOGA RIGHT FOOT
              0.000;   %% state == 11  PREPARING FOR SWITCHING
              10.00;   %% state == 12  LOOKING FOR CONTACT 
              0.000];  %% state == 13  TRANSITION INIT POSITION
          
% smoothing time joints
smTimeJoints = [0.5;   %% state ==  1  TWO FEET BALANCING 
                0.5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                0.5;   %% state ==  3  LEFT FOOT BALANCING 
                0.5;   %% state ==  4  YOGA LEFT FOOT
                0.5;   %% state ==  5  PREPARING FOR SWITCHING
                0.5;   %% state ==  6  LOOKING FOR CONTACT 
                0.5;   %% state ==  7  TRANSITION INIT POSITION
                0.5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                0.5;   %% state ==  9  RIGHT FOOT BALANCING 
                0.5;   %% state == 10  YOGA RIGHT FOOT
                0.5;   %% state == 11  PREPARING FOR SWITCHING
                0.5;   %% state == 12  LOOKING FOR CONTACT 
                0.5];  %% state == 13  TRANSITION INIT POSITION
            
% smoothing time CoM
smTimeCoM = [0.5;   %% state ==  1  TWO FEET BALANCING 
             0.5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
             0.5;   %% state ==  3  LEFT FOOT BALANCING 
             0.5;   %% state ==  4  YOGA LEFT FOOT
             0.5;   %% state ==  5  PREPARING FOR SWITCHING
             0.5;   %% state ==  6  LOOKING FOR CONTACT 
             0.5;   %% state ==  7  TRANSITION INIT POSITION
             0.5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
             0.5;   %% state ==  9  RIGHT FOOT BALANCING 
             0.5;   %% state == 10  YOGA RIGHT FOOT
             0.5;   %% state == 11  PREPARING FOR SWITCHING
             0.5;   %% state == 12  LOOKING FOR CONTACT 
             0.5];  %% state == 13  TRANSITION INIT POSITION
          
% smoothing time gains
smTimeGains = [0.5;   %% state ==  1  TWO FEET BALANCING 
               0.5;   %% state ==  2  COM TRANSITION TO LEFT FOOT
               0.5;   %% state ==  3  LEFT FOOT BALANCING 
               0.5;   %% state ==  4  YOGA LEFT FOOT
               0.5;   %% state ==  5  PREPARING FOR SWITCHING
               0.5;   %% state ==  6  LOOKING FOR CONTACT 
               0.5;   %% state ==  7  TRANSITION INIT POSITION
               0.5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
               0.5;   %% state ==  9  RIGHT FOOT BALANCING 
               0.5;   %% state == 10  YOGA RIGHT FOOT
               0.5;   %% state == 11  PREPARING FOR SWITCHING
               0.5;   %% state == 12  LOOKING FOR CONTACT 
               0.5];  %% state == 13  TRANSITION INIT POSITION
          
% time the robot should stay for each yoga reference
% smoothing time gains
SM.t_yoga = [3.0;   %% YOGA 1
             3.0;   %% YOGA 2
             3.0;   %% YOGA 3
             3.0;   %% YOGA 4
             3.0;   %% YOGA 5
             3.0;   %% YOGA 6
             3.0;   %% YOGA 7
             3.0];  %% YOGA 8
               
SM.t_treshold            = t_treshold(state);
SM.f_treshold            = f_treshold(state);
SM.smoothingTimeJoints   = smTimeJoints(state);
SM.smoothingTimeGains    = smTimeGains(state);
SM.smoothingTimeCoM      = smTimeCoM(state);
       
%% Invert references for yoga movements while balancing on right foot
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

%% Update contact constraints
if sum(MODEL.CONFIG.feet_on_ground) == 2
    % two feet balancing
    SM.constraintLinkNames = {'l_sole','r_sole'};
    
elseif MODEL.CONFIG.feet_on_ground(1) == 1 && MODEL.CONFIG.feet_on_ground(2) == 0
    % left foot balancing
    SM.constraintLinkNames = {'l_sole'};
    
elseif MODEL.CONFIG.feet_on_ground(1) == 0 && MODEL.CONFIG.feet_on_ground(2) == 1
    % right foot balancing
    SM.constraintLinkNames = {'r_sole'};
end
end
