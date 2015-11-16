function quatd = quatDerivative(quat, omega)
    if (length(quat) ~= 4)
        error('quatDerivative: %s', wbmErrMsg.WRONG_VEC_SIZE);
    end
    %if (length(omega) ~= 3)
    %    error('quatDerivative: %s', wbmErrMsg.WRONG_VEC_SIZE);
    %end

    K = 1;
    omegaCross = [0 -omega'; omega -skew(omega)];
    quatd = 0.5*omegaCross*quat + K*(1 - norm(quat))*quat;
end

