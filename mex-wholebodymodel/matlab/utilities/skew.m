function [ X ] = skew( x )
%SKEW Function to generate a 3X3 Skew Symmetric matrix out of a 3X1 vector.
%   Detailed explanation goes here
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];

end

