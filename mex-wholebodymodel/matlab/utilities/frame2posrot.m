function [pos,dcm] = frame2posrot(qT)
%FRAME2POSROT Converts the frame returned from standing pose wholeBodyModel into
%individual positions and the DCM rotation matrix
%   Arguments :
%   Normal Mode :
%               qT - (7 X 1) frame returned by whole body model
%               get-state (3 post followed by quaternion with real first)
%   Returns :
%               x - position x
%               y - position y
%               z - position z
%               dcm - Discrete Cosine Matrix (Rotation matrix 3 X 3)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it) - modified from
% matlab toolbox source
% Genova, Dec 2015

pos        = qT(1:3);
quaternion = qT(4:end);

% Assuming q = [q_real; q_vec]
dcm = quaternion2dcm(quaternion);

end
