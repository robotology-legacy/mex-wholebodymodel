function [T_bar,angles] = parametrization(rot)

%Z-Y-X rotation
phi   =  atan2(rot(3,2), rot(3,3));
theta = -asin(rot(3,1));
psi   =  atan2(rot(2,1),rot(1,1));

angles = [phi theta psi];

%Matrix which links the angular velocity with the derivative of euler
%angles

T_bar = [1     0           -sin(theta);
         0  cos(phi)   sin(phi)*cos(theta) ;
         0 -sin(phi)   cos(phi)*cos(theta)];
     
    
end