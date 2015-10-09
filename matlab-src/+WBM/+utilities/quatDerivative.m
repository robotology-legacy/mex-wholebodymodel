function dq = quatDerivative(q, omega)
    K = 1;
    omegaCross = [0 -omega'; omega -skew(omega)];
    dq = 0.5*omegaCross*q + K*(1 - norm(q))*q;
end

