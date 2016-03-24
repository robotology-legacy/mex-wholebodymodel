function [basePose,jointAngles,baseVelocity,jointsVelocity] = stateDemux(state,param)
%#codegen
    % state  
    % state(1:3)               =  I_p_b,     i.e. base position w.r.t. inertial frame
    % state(4:7)               =  Q_b,       i.e. quaternion representing base orientation w.r.t. base frame. More precisely, dot(Q_b) = -0.5 SB(b_omega_b)Q_b
    % state(8: 8+nDof-1)       =  qj,        i.e. joint angles
    % state(8+nDof:10+nDof)    =  I_v_b,     i.e. base linear velocity w.r.t. inertial frame
    % state(11+nDof:13+nDof)   =  I_omega_b, i.e. base angular velocity w.r.t. inertial frame
    % state(14+nDof:13+2*nDof) =  qjDot,     i.e. joint angles

    
    nDof            = param.ndof;
    I_p_b           = state(1:3);               
    Q_b             = state(4:7)';
    jointAngles     = state(8:8+nDof-1);
    baseVelocity    = state(8+nDof:13+nDof)';
    jointsVelocity  = state(14+nDof:13+2*nDof)';

    basePose        = zeros(4);
    
    if      param.demux.baseOrientationType == 0
        

        I_R_b       = quaternion2dcm(Q_b);
        basePose    = [I_R_b,I_p_b';zeros(1,3),1];
        
     elseif param.demux.baseOrientationType == 1
         
        basePose(1:3,4)  = I_p_b;
        basePose(:,1)    = Q_b;
        
    end

        
end

