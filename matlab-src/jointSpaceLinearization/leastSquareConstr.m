function [Kx,Kn] = leastSquareConstr(Ax,Bx,An,Bn,Kdes,params)

% separate the linear and angular momentum
Al  = Ax(:,1:3);
Bl  = Bx(1:3,:);
Aw  = Ax(:,4:6);
Bw  = Bx(4:6,:);

gainsPCoM = params.gainsPCoM;
impedances = params.impedances;
gainMomentum = params.gainPhi;
ndof = params.ndof; 

% Generate the desired shape for the matrices
tol = 0.1;

Kl  = @(X) [abs(X(1))+tol     0             0  ;
               0         abs(X(2))+tol      0  ;
               0              0       abs(X(3))+tol];
           
           
Kw  = @(X) [abs(X(4))+tol      0          0;
            X(5)       abs(X(6))+tol      0;
            X(7)           X(8)      abs(X(9))+tol];

        
function K = Kp(X)
        
 g  = 10; 
tol = 0.1;

    for kk = 1:25
            
        for jj = 1:25
                
                if kk>jj
                    
                    K(kk,jj) = X(g);
                    g = g+1;
       
                elseif kk == jj
                    
                    K(kk,jj) = abs(X(g))+tol;
                    g = g+1;
                else
                    K(kk,jj) = 0;
                end
         end
    end
end
         
    
%solution

  
 func = @(X) Kdes-Al*Kl(X)*Bl-Aw*(Kw(X)*Kw(X)')*Bw-An*(Kp(X)*Kp(X)')*Bn;
 
 
  OPTIONS = optimset('Algorithm','levenberg-marquardt');
  
  X0(1:3) = diag(gainsPCoM);
  
  Uw = chol(gainMomentum*eye(3));
  
  Up = chol(diag(impedances));
  
  Uw = Uw';
  Up = Up';
  
  g =4;
  
  for kk=1:3
      
      for jj = 1:3
          
          if kk>=jj
              
              X0(g)=Uw(kk,jj);
              g = g+1;
              
          end
      end
  end
  
  
  g = 10;
  
   for kk=1:ndof
      
      for jj = 1:ndof
          
          if kk>=jj
              
              X0(g)=Up(kk,jj);
              g = g+1;
              
          end
      end
  end
  
  
%     x = fsolve(func,1*ones((9+ndof*(ndof+1)/2)),OPTIONS);
  x = lsqnonlin(func,X0,[],[],OPTIONS);
        
  Kx = [Kl(x) zeros(3); zeros(3) Kw(x)*Kw(x)'];
  
  Kn = Kp(x)*Kp(x)';
  
end
  