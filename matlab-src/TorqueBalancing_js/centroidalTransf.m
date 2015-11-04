function [T, dT] = centroidalTransf(xcom,x_b,dxcom,dx_b,M)
% centroidalTransf
% converts the normal floating base frame of reference to 
% the centroidal frame of reference
%% T calculation
r       = xcom - x_b;

X       = [eye(3),Sf(r)';
          zeros(3),eye(3)];
      
Mbj     = M(1:6,7:end);
Mb      = M(1:6,1:6);
Js      = X*(Mb\Mbj);

ndof    = size(Mbj,2);

T       = [X,Js;
          zeros(ndof,6),eye(ndof)];
          
%% time derivative of T
dr      = dxcom - dx_b;
mdr     = M(1,1)*dr;

dX      = [zeros(3),Sf(dr)';
           zeros(3),zeros(3)];

dMb     = [zeros(3),Sf(mdr)';
           Sf(mdr),zeros(3)];

invMb   =  eye(6)/Mb;
inv_dMb = -invMb*dMb*invMb;

dJs     = dX*(Mb\Mbj) + X*inv_dMb*Mbj;
 
dT      = [dX,dJs;
         zeros(ndof,6),zeros(ndof)];
        
end