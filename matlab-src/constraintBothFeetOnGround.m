function [ JTransf ] = constraintBothFeetOnGround(tau, MInv, H)
%CONSTRAINTBOTHFEETONGROUND The combined constraint force due to having
%both feet on the ground
%   Detailed explanation goes here

   % persistent Jleft
   % persistent Jright
   
   constraintLink1 = 'l_sole';
   constraintLink2 = 'r_sole';
    
    Jleft = wholeBodyModel('jacobian',constraintLink1);
    Jright = wholeBodyModel('jacobian',constraintLink1);
    
   % M = wholeBodyModel('mass-matrix');
   % MInv = M^(-1);
    
   % H = wholeBodyModel('generalised-force');
    
    djdq1 = wholeBodyModel('djdq',constraintLink1);
    djdq2 = wholeBodyModel('djdq',constraintLink2);
    
    J = [Jleft;Jright];
    
    djdq = [djdq1;djdq2];
    
    f = ( J * MInv * J') * ( (J * MInv * ( H  + [zeros(6,1);tau])) - djdq);
    
    JTransf = J'*f;
end

