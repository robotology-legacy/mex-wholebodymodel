function [Kx,Kn] = leastSquareConstr(Ax,Bx,An,Bn,Kdes,params)
%LEASTSQUARECONSTR solves a non-linear least squares problem in order to 
%                  perform a gains optimization on the linearized system 
%                  dynamics of the robot iCub.
%   LEASTSQUARECONSTR uses the MATLAB function LSQNONLIN to solve the
%   problem: min(|Kdes-F(Kx,Kn)|^2). It takes into account the following
%   constraints on the gains matrices: Kx should be block diagonal; Kx(1:3,1:3) 
%   should be diagonal; Kx(4:6,4:6) should be symmetric and positive definite. 
%   Kn should be symmetric and positive definite. The symmetry and positive
%   definiteness constraints are ensured by means of Cholesky
%   decomposition.
%    
%   [Kx,Kn] = LEASTSQUARECONSTR(Ax,Bx,An,Bn,Kdes,params) takes as input the
%   pre and post multipliers of the gains matrices in the linearized
%   system's dynamics, Ax,Bx,An,Bn. Kdes is the optimization objective,
%   while params is a structure containing all the utility parameters.
%
%   The output are the optimized gains matrices, Kx [6x6] and Kn
%   [ndof x ndof].
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% setup the initial parameters
selPos       = strcmp(params.matrixSelector,'position');
selVel       = strcmp(params.matrixSelector,'velocity');
ndof         = params.ndof;
params.gPost = 10;
%gLin        = 1;
gAng         = 4;

if selPos == 1

gainsL = params.OldGains.PosGainsMom(1:3,1:3);
gainsW = params.OldGains.PosGainsMom(4:6,4:6);
gainsP = params.OldGains.KSdes;

elseif selVel == 1

gainsL = params.OldGains.VelGainsMom(1:3,1:3);
gainsW = params.OldGains.VelGainsMom(4:6,4:6);
gainsP = params.OldGains.KDdes;    
end

%% Separate the linear and angular momentum
ALin  = Ax(:,1:3);
BLin  = Bx(1:3,:);
AAng  = Ax(:,4:6);
BAng  = Bx(4:6,:);

%% INITIAL CONDITIONS
% linear momentum gains
X0(1:3) = diag(gainsL);

% angular momentum gains
Uw = chol(gainsW);
Uw = Uw';  
 
for kk=1:3
   
for jj = 1:3
          
     if kk>=jj
              
     X0(gAng) = Uw(kk,jj);
     gAng     = gAng+1;
     end
end
end

% postural gains
Up    = chol(gainsP); 
Up    = Up';
gPost = params.gPost;

for kk=1:ndof
      
for jj = 1:ndof
          
     if kk>=jj
              
     X0(gPost) = Up(kk,jj);
     gPost     = gPost+1;
     end
end
end
  
%% Optimization
F       = @(X) Kdes-(ALin*Kl(X,params)*BLin + AAng*(Kw(X,params)*Kw(X,params)')*BAng + An*(Kp(X,params)*Kp(X,params)')*Bn);
 
OPTIONS = optimset('Algorithm','levenberg-marquardt'); 
x       = lsqnonlin(F,X0,[],[],OPTIONS);


%% Gains matrices
Kx = [Kl(x,params) zeros(3); zeros(3) Kw(x,params)*Kw(x,params)'];  
Kn = Kp(x,params)*Kp(x,params)';
  
end

%% INTERNAL FUNCTIONS
function Matr = Kl(X,params)
% generate the linear momentum matrix
PosToll = params.PosToll;
Matr    = [abs(X(1))+PosToll     0             0  ;
              0         abs(X(2))+PosToll      0  ;
              0                  0       abs(X(3))+PosToll];
           
end           

function Matr = Kw(X,params)
PosToll = params.PosToll;
% generate the angular momentum matrix
Matr    = [abs(X(4))+PosToll      0              0;
               X(5)       abs(X(6))+PosToll      0;
               X(7)           X(8)      abs(X(9))+PosToll];

end
      
function Matr = Kp(X,params)
% generate the postural matrix        
gPost    = params.gPost; 
PosToll  = params.PosToll;
ndof     = params.ndof;
Matr     = zeros(ndof);

for kk = 1:ndof
            
for jj = 1:ndof
                
    if kk>jj
                    
     Matr(kk,jj) = X(gPost);
     gPost       = gPost+1;
       
    elseif kk == jj
                    
     Matr(kk,jj) = abs(X(gPost))+PosToll;
     gPost       = gPost+1;
    else
     Matr(kk,jj) = 0;
    end
end
end
end

  