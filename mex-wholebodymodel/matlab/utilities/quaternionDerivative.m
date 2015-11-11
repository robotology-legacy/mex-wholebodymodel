function [ qDot ] = quaternionDerivative( omega, q )
%QUATERNIONDERIVATIVE Function to compute the numerical derivative of a
%quaternion from angular velocity as a vector
%   Computes a derivate of quaternion using the classical form and includes
%   a stabilization term to make the vector sum equal to 1
%   The formula for the conversion from omega to the derivative of the 
%   quaternion comes from https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf 
%   page 101. 

K = 1;

omegaCross  = [0 -omega';omega -skew(omega)];

qDot = 0.5*omegaCross * q + K*(1-norm(q)) * q;

end

