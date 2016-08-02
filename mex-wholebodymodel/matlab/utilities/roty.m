%% Utility function for visualizeForwardDynamics.m
function R = roty(alpha)
   R = zeros(3, 3);
   R(2,2) =  1;
   R(1,1) =  cos(alpha);
   R(1,3) =  sin(alpha);
   R(3,1) = -sin(alpha);
   R(3,3) =  cos(alpha); 
end