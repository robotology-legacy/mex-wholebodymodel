%% Utility function for visualizeForwardDynamics.m

function P = fromMesh2sortedVector(x,y,z)
   x = x(:)';
   y = y(:)';
   P = [x;y];
   if nargin == 3
       z = z(:)';
       P = [P;z];
   end
end