function [ q ] = convertAxisAngle2Quaternion( aa )
%CONVERTAXISANGLE2QUATERNION Summary of this function goes here
%   Detailed explanation goes here

theta = aa(4);
q = [cos(theta/2) ; sin(theta/2).*aa(1:3)];
q = q./norm(q);

end

