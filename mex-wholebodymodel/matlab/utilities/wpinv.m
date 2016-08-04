function [G,F] = wpinv(A,W)
% WPINV - Compute weighted pseudoinverse.
%
%  [G,F] = wpinv(A,W)
%
% Computes the optimal solution x = Gy + Fx0 to the least squares
% problem
%
%  min ||W(x-x0)||  subj. to  Ax = y
%   x
%
% See also PINV.

[~,n] = size(A);

if nargin < 2
    W = eye(n);
end

% Thesis, Lemma B.1
G = W\pinvDamped(A/W,1e-4);
F = eye(n) - G*A;
end