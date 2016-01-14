function [x,y,z,R] = quat2rot(qT)

%q_ang = acos(q(4))*2;
%rx = q(5)./sqrt(1-q(4).*q(4));
%ry = q(6)./sqrt(1-q(4).*q(4));
%rz = q(7)./sqrt(1-q(4).*q(4));

x = qT(1);
y = qT(2);
z = qT(3);

q = qT(4:end);

% Assuming q = [q_real; q_vec]
% The matrix Q(q) defined in (13.16)
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   R = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2)-1];

% R = zeros(3); %vrrotvec2mat([rx(1) ry(1) rz(1) q_ang(1)]);
% R=0.25*fliplr(R);
% R=0.25*R;

end
