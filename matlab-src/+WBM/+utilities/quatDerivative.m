function quatd = quatDerivative(quat, omega)
    if (size(quat,1) ~= 4) || (size(omega,1) ~= 3)
        error('quatDerivative: %s', wbmErrMsg.WRONG_VEC_DIM);
    end
    
    K = 1;
    
    % Create the conjugate product matrix "Omega" of the angular velocity omega:
    % Note: Further details about the product matrix (operator) Omega
    % can be taken from
    %       * https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf,
    %         page 101, 
    %       * http://soliton.ae.gatech.edu/people/obauchau/publications/Bauchau+Trainelli03.pdf,
    %         page 16-18, formulas (51) & (73),
    %       * and http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf,
    %         page 19, formulas (97) & (98).
    % Furthermore, this is faster than an array-concatenation ...
    Omega_op = zeros(4,4);
    Omega_op(1,2:4)   = -omega';
    Omega_op(2:4,1)   = omega;
    Omega_op(2:4,2:4) = -skew(omega); % (skew-symmetric) cross product matrix
    
    % This calculation of the quaternion derivative has additionally a
    % computational "hack" (K*ε*q) to compute the derivative in the case
    % that the quaternion is not normalized, s.t. the vector sum is still
    % equal to 1. The factor ε will be 0 if the quaternion is unitary.
    % For further details see:
    %   http://mathworks.com/help/aeroblks/customvariablemass6dofquaternion.html
    %   and https://www.physicsforums.com/threads/quaternion-derivative.706475
    quatd = 0.5*Omega_op*quat + K*(1 - norm(quat))*quat;
end

