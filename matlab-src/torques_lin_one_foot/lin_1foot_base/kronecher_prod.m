function [K1_new,K2_new,K3_new] = kronecher_prod(C,R1,R2,S1,S2,lparam,name)

a1 = strcmp(name,'position');
 
 if a1 == 1
     
 A1 = kron(R2.',R1);
 
 A2 = kron(S2.',S1);
 
 for kk = 1:sqrt(size(A2,1))
     
     ei=zeros(sqrt(size(A2,1)),1);
     ei(kk) =1;
     
 Ei=ei*ei.';
 E(:,kk)=Ei(:);
 
 end
 
 
 reg = 1e-15;
 A_kron = (E.')*[A1 A2];
 matrix = [A1 A2 -E];
 pinvm = pinv(matrix, reg);
 null = eye(size(pinvm,1))-pinvm*matrix;
 size(null)
 sum(sum(abs(null)))
 nulldiag = null(size([A1 A2],2)+1:end);

 delta2 = 1*ones(size(null,2),1);
%  size(delta2)
 
 vett2 = null;
 size(vett2)
sum(sum(abs( vett2(:,635:end))))
 

size(E)
coso=[[A1 A2]*vett2(1:634,:)*delta2-E*vett2(635:end,:)*delta2]
sum(sum(abs(coso)))


 pinvA_kron  = pinv(A_kron,reg); 
 
%  cvett = C(:);
%  cvett = -15*ones(25,1);
cvett = A_kron(:,3);

%  vett = pinvA_kron*cvett;
vett =vett2(1:634,:)*delta2;
lambda = sum(sum(abs(vett2(635:end,:))))

polipalle

Kg_new     = zeros(3);     
Kp_new     = zeros(3);
Kimp_new   = zeros(25);

Kimp_new_v = vett(10:end);
Kp_new_v   = vett(1:9);

g = 0;

for kk=1:3
    
    Kp_new(:,kk) = Kp_new_v(g+1:g+3);
    
    g=g+3;
    
end

g = 0;

for kk=1:25
    
    Kimp_new(:,kk) = Kimp_new_v(g+1:g+25);
    
    g=g+25;
    
end

K1_new = Kp_new;
K2_new = Kg_new;
K3_new = Kimp_new;

 end
 
%  aa = [A1 A2];
%  bb=[aa E*cvett];
%  [s,v,d]=svd(bb);
%  [ss,vv,dd]=svd(aa);
%  
%  [diag(v) diag(vv)]
%  
 [ eig(R1*K1_new*R2 + S1*K3_new*S2) lambda] 
% R1*K1_new*R2 + S1*K3_new*S2

%% 
a2 = strcmp(name,'velocity');
 
if a2 == 1

 m=lparam.m;
 
 R1_tilde1 = R1*[-m*eye(3);zeros(3)];
 R2_tilde1 = R2(1:3,:);
 
 R1_tilde2 = R1*[zeros(3); -eye(3)];
 R2_tilde2 = R2(4:6,:);
     
 A11 = kron(R2_tilde1.',R1_tilde1);
 
 A12 = kron(R2_tilde2.',R1_tilde2);
 
 A2 = kron(S2.',S1);
 
 A_kron = [A11 A12 A2];
 
 reg = 1e-5;
 pinvA_kron  = pinv(A_kron,reg);  

 cvett = C(:);
 vett = pinvA_kron*cvett;
     
 Kg_new   = zeros(3);
 Kd_new   = zeros(3);
 Kder_new = zeros(25);

Kg_new_v   = vett(10:18);     
Kder_new_v = vett(19:end);
Kd_new_v   = vett(1:9);

g = 0;

for kk=1:3
    
    Kd_new(:,kk)=Kd_new_v(g+1:g+3);
    Kg_new(:,kk)=Kg_new_v(g+1:g+3);
    
    g=g+3;
    
end

g = 0;

for kk=1:25
    
    Kder_new(:,kk)=Kder_new_v(g+1:g+25);
    
    g=g+25;
    
end

K1_new = Kd_new;
K2_new = Kg_new;
K3_new = Kder_new;

 end
 
end