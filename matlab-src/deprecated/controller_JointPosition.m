% function [tau,CoMError,xDDcomStar] = controller_BalancingTorque(q,v,  IntErrorCoM,prm)
function [tau,CoMError,xDDcomStar] = controller_JointPosition(qj,qjDot,t,params)

    %PINV_TOL = 1e-5;

    %gravAcc        = 9.81;

    kp = params.kp;
    kd = params.kd;

    qjDes = params.qjDes(t);
    qjDotDes = params.qjDotDes(t);

    tau = kp*(qjDes - qj) + kd*(qjDotDes - qjDot);
    CoMError = zeros(3,1);
    xDDcomStar = zeros(3,1);
end

