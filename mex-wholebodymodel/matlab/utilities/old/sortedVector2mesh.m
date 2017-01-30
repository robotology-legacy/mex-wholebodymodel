%% Utility function for visualizeForwardDynamics.m
function P = sortedVector2mesh(x,Dx)
i1 = 1;
i2 = Dx;
for i =1:length(x)/Dx
    P(:,i) = x(i1:i2);
    i1 = i2 + 1;
    i2 = i2 + Dx;
end
end

