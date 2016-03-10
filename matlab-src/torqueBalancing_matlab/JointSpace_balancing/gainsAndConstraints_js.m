function gains = gainsAndConstraints_js (param)
%% gainsAndConstraints_js 
%  Generates the desired gains for both the joint and the CoM
%  dynamics.
%  The output is:
%
%  gains            this is a structure which contains both CoM and joints gains
%
%% Setup parameters
feet_on_ground   = param.feet_on_ground;
ndof             = param.ndof;

%% Two feet impedances
if sum(feet_on_ground)  == 2
    
       impTorso            = [ 50    50   50
                                0     0    0]; 
                           
       impArms             = [ 10    10   10   10  20  
                                0     0    0    0   0];
                        
       impLeftLeg          = [ 35   50    5   30   5  10
                                0    0    0    0   0   0]; 

       impRightLeg         = [35   50    5   30   5  10
                               0    0    0    0   0   0];                                                 
end

%% One foot impedances
if  sum(feet_on_ground) == 1

      impTorso            = [  20    20   20
                                0     0    0]; 

      impArms             = [ 13  13   13   5   13
                               0    0    0   0   0 ];

if param.feet_on_ground(1) == 1
    
     impLeftLeg          = [ 70   70  65  30  10  10
                              0    0   0   0   0   0]; 

     impRightLeg         = [ 20   20  20  10  10  10
                              0    0   0   0   0   0];
    
else
   
     impRightLeg         = [ 70   70  65  30  10  10
                              0    0   0   0   0   0]; 

     impLeftLeg          = [ 20   20  20  10  10  10
                              0    0   0   0   0   0];

end
end

%% Dampings and impedances
gains.impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
   
gains.dampings            = 0.5*ones(1,ndof);

if (size(gains.impedances,2) ~= ndof)
    
    error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');
    
end

end
