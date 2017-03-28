function [pinvM,NullSpace] = pinvWeight(M,Weight)
% pinvWeight computes the weighted pseudoinverse of a matrix M
%
%  [pinvM,NullSpace] = pinvWeight(M,Weight) solves the least squares
%  problem 
%  min ||Weight(x-x0)||  s.t.  Mx = y
%   x
%  the solution is x = pinvM(y) + NullSpace(x0) 
%
[~,n]     = size(M);
pinvM     = Weight\pinvDamped(M/Weight,1e-4);
NullSpace = eye(n) - pinvM*M;

end