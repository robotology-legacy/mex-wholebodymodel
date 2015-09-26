function [pos,dcm] = frame2posrot(qT)
%FRAME2POSROT Converts the frame returned from%for standing pose wholeBodyModel into
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

pos = qT(1:3);
%qin = qT(4:end)';
qt_b_mod_s = qT(4);
qt_b_mod_r = qT(5:end);

% Assuming q = [q_real; q_vec]

dcm = zeros(3,3);%,size(qin,1));

% dcm(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
% dcm(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
% dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
% dcm(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
% dcm(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
% dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
% dcm(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
% dcm(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
% dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;

dcm = eye(3) + 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;
end
