function [impedances, dampings] = gains (DOF, LEFT_RIGHT_FOOT_IN_CONTACT)
% gains 
% defines the gains for joints space balancing controller
% both impedances and damping at joints are calculated
%% Two feet impedances
if LEFT_RIGHT_FOOT_IN_CONTACT == 2
    
       impTorso            = [ 50    50   50
                                0     0    0]; 
                           
       impArms             = [ 10    10   10   10  20  
                                0     0    0    0   0];
                        
       impLeftLeg          = [ 35   50    2   30   2  10
                                0    0    0    0   0   0]; 

       impRightLeg         = [35   50    2   30   2  10
                               0    0    0    0   0   0];                                                 
end

%% One foot impedances
if  LEFT_RIGHT_FOOT_IN_CONTACT == 1

      impTorso            = [  20    20   20
                                0     0    0]; 

      impArms             = [ 13  13   13   5   13
                               0    0    0   0   0 ];

      impLeftLeg          = [ 70   70   65  30  10  10
                               0    0    0   0   0   0]; 

      impRightLeg         = [ 20   20  20  10  10  10
                               0    0   0   0   0   0];
end

%% damping
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
   
dampings            = 1*ones(1,DOF);
%dampings           = 2*sqrt(impedances);

if (size(impedances,2) ~= DOF)
    
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
    
end

end
