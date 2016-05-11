%% OneFootLinearization
% linearizes the joint space equations of motion of robot iCub when it's
% controlled with "stack of task" control approach. This version is for
% the robot balancing on one foot.
% Output:
%  
% Linearization     this is a structure containing all the parameters
%                   coming from the linearized system
%
% NewGains          this structure contains the gains obtained with the
%                   gains tuning procedure
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [Linearization, NewGains] = OneFootLinearization(params,gainsInit)    
%% Setup all the parameters
toll           = params.pinv_tol;
damp           = params.pinv_damp;
feet_on_ground = params.feet_on_ground;     
ndof           = params.ndof;
% feet jacobian
Jc             = params.JcInit;
% mass matrix
M              = params.MInit;
% matrix A at CoM
if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
x_sole   = params.PoseLFootQuatInit ;

elseif feet_on_ground(2) == 1 && feet_on_ground(1) == 0
    
x_sole   = params.PoseRFootQuatInit ;
end

CoM          = params.CoMInit;
posFoot      = x_sole(1:3);
xCoM         = CoM(1:3);
r            = posFoot-xCoM;
A            = [eye(3)   zeros(3);
                 skew(r)  eye(3) ];
m            = M(1,1);
invA         = eye(6)/A;
Mb           = M(1:6,1:6);
Mbj          = M(1:6,7:end);
Mjb          = M(7:end,1:6);
Mj           = M(7:end,7:end);
Jb           = Jc(1:6,1:6);
Jj           = Jc(1:6,7:end);
Mbar         = Mj - Mjb/Mb*Mbj;

% Mbar_inv   = Mbar'/(Mbar*Mbar' + damp*eye(size(Mbar,1)));
Mbar_inv     = eye(ndof)/Mbar;

%% Terms that compose the linearized joint space dynamics
Lambda             =  (Jj - Jb/Mb*Mbj)*Mbar_inv;
MultFirstTask      =  Jb/Mb*transpose(Jb)*invA;
pinvLambda         =  pinv(Lambda,toll);
FirstTask          =  pinvLambda*MultFirstTask;
NullLambda         =  eye(ndof) - pinvLambda*Lambda; 

%% Old gains
gainsPCoM                 = gainsInit.gainsPCoM;
gainsDCoM                 = gainsInit.gainsDCoM;
gainsPAngMom              = gainsInit.gainsPAngMom; 
gainsDAngMom              = gainsInit.gainsDAngMom;
impedances                = gainsInit.impedances;
dampings                  = gainsInit.dampings;

NewGains.impedances       = gainsInit.impedances; 
NewGains.dampings         = gainsInit.dampings;
NewGains.posturalCorr     = gainsInit.posturalCorr; 
NewGains.VelGainsMom      = [gainsInit.gainsDCoM zeros(3); zeros(3) gainsInit.gainsDAngMom];
NewGains.PosGainsMom      = [gainsInit.gainsPCoM zeros(3); zeros(3) gainsInit.gainsPAngMom];

%% Parameters from the cartesian task
% CoM jacobian
JCoM_total =  params.JCoMInit;
JCoM       =  JCoM_total(1:3,:);
JCoM_b     =  JCoM(:,1:6);
JCoM_j     =  JCoM(:,7:end);
% centroidal momentum jacobian
Jh         =  params.JhInit;
% angular momentum jacobian (decomposed)
Jw_b       =  Jh(4:6,1:6);
Jw_j       =  Jh(4:6,7:end); 

% conversion term between Nu_base and dqj, obtained from  the contact
% constraints equations
Nu_baseFrom_dqj     =  -(eye(6)/Jb)*Jj;

%% Analytical derivative with respect of joint position
xCoM_posDerivative  = JCoM_b*Nu_baseFrom_dqj + JCoM_j;

% centroidal orientation corrections to assure local stability
angularOrientation  = -(Jw_b*Nu_baseFrom_dqj + Jw_j);

Kimp                =  impedances*gainsInit.posturalCorr;
Kdamp               =  dampings*gainsInit.posturalCorr;

HDot_posDerivative  = [-m.*gainsPCoM*xCoM_posDerivative; gainsPAngMom*angularOrientation];

%% Analytical derivative with respect of joint velocity
dxCoM_velDerivative = JCoM_b*Nu_baseFrom_dqj + JCoM_j;
Hw_velDerivative    = Jw_b*Nu_baseFrom_dqj + Jw_j;

HDot_velDerivative  = [-m.*gainsDCoM*dxCoM_velDerivative; -gainsDAngMom*Hw_velDerivative];

%% Stiffness
KS      =  Mbar_inv*(FirstTask*HDot_posDerivative + NullLambda*Kimp);

%% Damping
KD      =  Mbar_inv*(FirstTask*HDot_velDerivative + NullLambda*Kdamp);

%% Analysis on the linearized system
% State matrix verification
if params.linearize_for_stability_analysis == 1

A_state_old     = [zeros(ndof) eye(ndof);
                    -KS          -KD];

ReigAstate_old  = -real(eig(A_state_old));                

toleig          = 1e-5;
flag            = 0;
logicEigOld     = ReigAstate_old<toleig;

if sum(logicEigOld)>0
    
    flag = 1;

end

if flag == 1

    disp('Warning: the linearized state dynamics is NOT asymptotically stable')
    
else
    
    disp('The linearized state dynamics is asymptotically stable')
        
end

end

% Parameters for visualization
Linearization.KS    = KS;
Linearization.KD    = KD;
Linearization.KDdes = gainsInit.KDdes;
Linearization.KSdes = gainsInit.KSdes;

%% Gains tuning procedure
if params.linearize_for_gains_tuning == 1
    
[Kpx,Kdx,Kpn,Kdn,KSn,KDn] = gainsTuning(Mbar_inv,FirstTask,m,xCoM_posDerivative,angularOrientation,NullLambda,Mbar,dxCoM_velDerivative,...
                                        Hw_velDerivative,gainsInit,ndof,toll,damp);


% Parameters for visualization                                        
Linearization.KSn   = KSn;
Linearization.KDn   = KDn;
Linearization.Kpn   = Kpn;
Linearization.Kdn   = Kdn;
Linearization.Kpx   = Kpx;
Linearization.Kdx   = Kdx;

% New gains matrices
NewGains.impedances    = Kpn; 
NewGains.dampings      = Kdn;
NewGains.VelGainsMom   = Kdx;
NewGains.PosGainsMom   = Kpx;

% Verify the new state matrix
A_state_new     = [zeros(ndof) eye(ndof);
                    -KSn        -KDn];
      
A_state_desired = [zeros(ndof)             eye(ndof);
                  -gainsInit.KSdes   -gainsInit.KDdes];

ReigAstate_des       = -real(eig(A_state_desired));                
ReigAstate_new       = -real(eig(A_state_new));

toleig               = 1e-5;
flag                 = [0;0];

logicNewEig   = ReigAstate_new<toleig;
if sum(logicNewEig)>0
    
    flag(1) = 1;

end

logicNewEig   = ReigAstate_des<toleig;
if sum(logicNewEig)>0
    
    flag(2) = 1;

end

if flag(1) == 1

    disp('Warning: the new linearized state dynamics after gains tuning is NOT asymptotically stable')
    
elseif flag(2) == 1
    
    disp('Warning: your desired linearized state dynamics is NOT asymptotically stable')
    

elseif sum(flag) == 0
    
    disp('The linearized state dynamics after gains tuning is asymptotically stable')
        
end

end

end
