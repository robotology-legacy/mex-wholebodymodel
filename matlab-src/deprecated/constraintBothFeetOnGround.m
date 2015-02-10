function [ fTot,J ] = constraintBothFeetOnGround(qj,tau, M, H)
%CONSTRAINTBOTHFEETONGROUND The combined constraint force due to having
%both feet on the ground
%   Detailed explanation goes here

   % persistent Jleft
   % persistent Jright
   
   constraintLink1 = 'l_sole';
   constraintLink2 = 'r_sole';
    
    Jleft  = reshape(wholeBodyModel('jacobian',constraintLink1),31,6)';
    Jright = reshape(wholeBodyModel('jacobian',constraintLink2),31,6)';
    
   % M = wholeBodyModel('mass-matrix');
   % MInv = M^(-1);
    
   % H = wholeBodyModel('generalised-force');
    
    djdq1 = wholeBodyModel('djdq',constraintLink1);
    djdq2 = wholeBodyModel('djdq',constraintLink2);
    
    J = [Jleft;Jright];
    
    JMInv = J/M;
    djdq = [djdq1;djdq2];
    
    fTot = ( JMInv * J')\ ( (-JMInv * ( -H  + [zeros(6,1);tau])) - djdq);
    
    %JTransf = J'*f;
end

