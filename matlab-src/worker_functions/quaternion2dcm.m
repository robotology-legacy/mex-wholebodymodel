function [dcm] = quaternion2dcm(quat)
%QUATERNION2DCM Converts a quaternion (real/imaginary) to direction cosine matrix (aka Rotation matrix)
%   Arguments :
%   Normal Mode :
%               quat - (4 X 1) vector representing the quaternion with real/imaginary serialization
%   Returns :
%               dcm - Discrete Cosine Matrix (Rotation matrix 3 X 3)
%

% notice that we are using real/imaginary serialization
qt_b_mod_s = quat(1);
qt_b_mod_r = quat(2:end);

% For more background on this formula, please check
% https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf
% Page 101, Formula 3.8
%
dcm = eye(3) + 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;

end
