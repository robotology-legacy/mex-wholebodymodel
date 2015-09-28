classdef iCubWBMState
    properties
       q_j
       dq_j
       x_b      = zeros(3,1);
       qt_b     = zeros(4,1);
       dx_b     = zeros(3,1);
       omega_b  = zeros(3,1);
    end
end