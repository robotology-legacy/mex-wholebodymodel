function [T, dT] = centroidalTransformationT_TDot(xCoM,x_b,dxCoM,dx_b,M)
% centroidalTransformationT_TDot
% converts the normal floating base frame of reference to 
% the centroidal frame of reference
%% T calculation
r       = xCoM - x_b;

X       = [eye(3),skew(r)';
          zeros(3),eye(3)];
      
Mbj     = M(1:6,7:end);
Mb      = M(1:6,1:6);
Js      = X*(Mb\Mbj);

ndof    = size(Mbj,2);

T       = [X,Js;
          zeros(ndof,6),eye(ndof)];
          
%% time derivative of T
dr      = dxCoM - dx_b;
mdr     = M(1,1)*dr;

dX      = [zeros(3),skew(dr)';
           zeros(3),zeros(3)];

dMb     = [zeros(3),skew(mdr)';
           skew(mdr),zeros(3)];

inv_dMb = -Mb\dMb/Mb;

dJs     = dX*(Mb\Mbj) + X*inv_dMb*Mbj;
 
dT      = [dX,dJs;
          zeros(ndof,6),zeros(ndof)];
        
end
