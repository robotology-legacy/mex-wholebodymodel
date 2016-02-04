function [KCoM_lin,KCoM_ang,Kpost] = KronecherProd(desMatrix,R1,R2,S1,S2,linParam,name)
%% KronecherProd 
%  uses the Kronecher product to generate as close as possible a desired
%  stiffness or damping matrix by means of CoM and postural gains.
%  The output are:
% 
%  KCoM_lin [3 x 3]        which are the gains related to CoM linear position and
%                          velocity
%
%  KCoM_ang [3 x 3]        which are the gains related to CoM angular velocity
%
%  Kpost    [ndof x ndof]  which are the gains related to the joints
%
%% Code for generating the stiffness matrix
reg  = 1e-5;
ndof = linParam.ndof; 
a1   = strcmp(name, 'position');

if a1 == 1
    
    product1   = kron(R2',R1);
    product2   = kron(S2',S1);
    
    Matrix     = [product1 product2];
    pinvMatrix = pinv(Matrix, reg);
    
    desVett    = desMatrix(:);
    
    gainsVett  = pinvMatrix*desVett;
    
    KCoM_ang   = zeros(3);     
    KCoM_lin   = zeros(3);
    Kpost      = zeros(ndof);

    KpostVett      = gainsVett(10:end);
    KCoM_linVett   = gainsVett(1:9);

% CoM position gains    
    g = 0;
  
for kk=1:3
    
    KCoM_lin(:,kk) = KCoM_linVett(g+1:g+3);
    
    g=g+3;
    
end

% Joint position gains
    g = 0;

for kk=1:ndof
    
    Kpost(:,kk) = KpostVett(g+1:g+ndof);
    
    g=g+ndof;
    
end

end

%% Code for generating the damping matrix
a2   = strcmp(name, 'velocity');
m    = linParam.m;  

if a2 == 1
      
 % CoM linear velocity gains multipliers    
 R1_tilde1   = R1*[-m*eye(3); zeros(3)];
 R2_tilde1   = R2(1:3,:);
 
 % CoM angular velocity gains multipliers    
 R1_tilde2   = R1*[zeros(3); -eye(3)];
 R2_tilde2   = R2(4:6,:);
     
 product1    = kron(R2_tilde1',R1_tilde1);   
 product2    = kron(R2_tilde2',R1_tilde2);
 product3    = kron(S2',S1);
 
 Matrix      = [product1 product2 product3];
 pinvMatrix  = pinv(Matrix,reg);  

 desVett    = desMatrix(:);
    
 gainsVett  = pinvMatrix*desVett;
    
 KCoM_ang   = zeros(3);     
 KCoM_lin   = zeros(3);
 Kpost      = zeros(ndof);

 KpostVett      = gainsVett(19:end);
 KCoM_angVett   = gainsVett(10:18);
 KCoM_linVett   = gainsVett(1:9);

% CoM position gains    
    g = 0;
  
for kk=1:3
    
    KCoM_lin(:,kk) = KCoM_linVett(g+1:g+3);
    KCoM_ang(:,kk) = KCoM_angVett(g+1:g+3);
    
    g=g+3;
    
end

% Joint position gains
    g = 0;

for kk=1:ndof
    
    Kpost(:,kk) = KpostVett(g+1:g+ndof);
    
    g=g+ndof;
    
end

end

end
 