classdef wbmStateParams
    properties
        % postions & orientation:
        x_b      = zeros(3,1);
        qt_b     = zeros(4,1);
        q_j
        % velocities:
        dx_b     = zeros(3,1);
        omega_b  = zeros(3,1);
        dq_j
    end
end