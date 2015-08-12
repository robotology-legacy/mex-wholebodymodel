 %#codegen
function R = Rf(u)
    theta = u(4);
    u     = u(1:3); 
    R = eye(3) + sin(theta)*Sf(u) + (1 - cos(theta))*Sf(u)*Sf(u);
end

