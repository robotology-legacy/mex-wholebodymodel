function [T_bar,angles] = parametrization(rot_matr)
%% parametrization
% Computes the Z-Y-X parametrization from a rotation matrix 
phi   =  atan2(rot_matr(3,2),rot_matr(3,3));
theta = -asin(rot_matr(3,1));
psi   =  atan2(rot_matr(2,1),rot_matr(1,1));

angles = [phi theta psi];

%Matrix which links the angular velocity with the derivative of euler
%angles

T_bar = [1     0           -sin(theta);
         0  cos(phi)   sin(phi)*cos(theta) ;
         0 -sin(phi)   cos(phi)*cos(theta)];
         
end