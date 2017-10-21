classdef wbmStateParams
    properties
        % positions & orientation:
        x_b@double     matrix
        qt_b@double    matrix
        q_j@double     matrix
        % velocities:
        dx_b@double    matrix
        omega_b@double matrix
        dq_j@double    matrix
    end
end