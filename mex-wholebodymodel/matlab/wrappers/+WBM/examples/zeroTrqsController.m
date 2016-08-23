function [tau, ctrl_data] = zeroTrqsController(vec_sz)
    tau = zeros(vec_sz); % no torque forces, only the gravity and external forces influencing the system.
    ctrl_data = struct();

    % ctrl_data.test_1 = ones(25,1)*0.5;
    % ctrl_data.test_2 = ones(1,12)*0.25;
    % ctrl_data.test_4 = 5;
    % ctrl_data.test_3 = ones(10,10);
end
