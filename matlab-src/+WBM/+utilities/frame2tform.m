function H frame2tform(vqT)
	% get the translation and the orientation of the frame ...
    [p, R] = frame2posRotm(vqT);
	% create the homogeneous transformation matrix H:
	H = zeros(4,4);
	H(1:3,1:3) = R; % rotation
	H(1:3,4)   = p; % translation
	H(4,1:3)   = zeros(1,3);
	H(4,4)     = 1;
end
