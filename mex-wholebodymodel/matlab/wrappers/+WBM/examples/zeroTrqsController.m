function [tau, ctrl_data] = zeroTrqsController(vec_sz)
    tau = zeros(vec_sz); % no internal torque forces, only the gravity and external forces are affecting the system.
    ctrl_data = struct();
end
