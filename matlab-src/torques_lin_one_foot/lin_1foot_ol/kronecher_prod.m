function [K1_new,K2_new] = kronecher_prod(C,R1,R2,S1,S2,lparam)


 m=lparam.m;
 
 R1_tilde1 = R1;
 R2_tilde1 = R2(1:3,:);
     
 A11 = kron(R2_tilde1.',R1_tilde1);
 
 A2 = kron(S2.',S1);
 
 A_kron = [A11 A2];
 
 reg = 1e-5;
 pinvA_kron  = pinv(A_kron,reg);  

 cvett = C(:);
 vett = pinvA_kron*cvett;
     
 Kd_new   = zeros(3);
 Kder_new = zeros(25);
     
 Kder_new_v = vett(10:end);
 Kd_new_v   = vett(1:9);

 g = 0;

for kk=1:3
    
    Kd_new(:,kk)=Kd_new_v(g+1:g+3);
    
    g=g+3;
    
end

 g = 0;

for kk=1:25
    
    Kder_new(:,kk)=Kder_new_v(g+1:g+25);
    
    g=g+25;
    
end

K1_new = Kd_new;
K2_new = Kder_new;


end
