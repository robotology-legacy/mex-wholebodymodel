function [T_bar,angles] = parametrization(rot)

% X-Y-Z parametrization 
C2 =  rot(3,3); 
S2 =  sqrt(1-C2.^2);

C1 = -rot(2,3)/S2;
C3 =  rot(3,2)/S2;

S1 =  rot(1,3)/S2;
S3 =  rot(3,1)/S2;

t1    =  atan2(S1,C1);
t2    =  atan2(S2,C2);
t3    =  atan2(S3,C3);

angles =[t1 t2 t3];

T_bar = [sin(t2)*sin(t3)  cos(t3)    0;
         sin(t2)*cos(t3) -sin(t3)    0;
              cos(t2)       0        1];     
    
end