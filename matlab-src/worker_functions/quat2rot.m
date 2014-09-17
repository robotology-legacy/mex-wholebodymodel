function [x,y,z,R] = quat2rot(q)

q_ang = acos(q(4))*2;
rx = q(5)./sqrt(1-q(4).*q(4));
ry = q(6)./sqrt(1-q(4).*q(4));
rz = q(7)./sqrt(1-q(4).*q(4));

x = q(1);
y = q(2);
z = q(3);


R = vrrotvec2mat([rx(1) ry(1) rz(1) q_ang(1)]);
% R=0.25*fliplr(R);
R=0.25*R;