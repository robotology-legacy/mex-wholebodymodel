function [ qDot ] = quaternionDerivative( omega, q )
%QUATERNIONDERIVATIVE Function to compute the numerical derivative of a
%quaternion from angular velocity as a vector
%   Computes a derivate of quaternion using the classical form and includes
%   a stabilization term to make the vector sum equal to 1
%   The formula for the conversion from omega to the derivative of the
%   quaternion comes from https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf
%   page 101.

K = 10;

if abs(1-norm(q)) > 0.5
    error('Quaternion left SO(3): abs((1-norm(q))) > 0.5') 
end

if abs(1-norm(q)) > 1e-3
    disp('Quaternion is leaving SO(3): abs((1-norm(q))) > 1e-3');
end

omegaCross  = [0 -omega';omega -skew(omega)];
qDot        = 0.5*omegaCross * q + K*(1-norm(q)) * q;

end

