function [ qDot ] = quaternionDerivative( omega, q )
%QUATERNIONDERIVATIVE Function to compute the numerical derivative of a
%quaternion from angular velocity as a vector
%   Computes a derivate of quaternion using the classical form and includes
%   a numerical 'hack' to make the vector sum equal to 1

K = 1;

omegaCross  = [0 -omega';omega -skew(omega)];

qDot = 0.5*omegaCross * q + K*(1-norm(q)) * q;


end

