function [der] = der_Kder(j)

global conv_r_leg
if j< 14 || j>19
    
    der = zeros(25,1);
    
    der(j) = 1;
    
else
    
    der = zeros(25,1);
    
    der(j)=1;
    
    der(20:end) = conv_r_leg(:,j);
    
end