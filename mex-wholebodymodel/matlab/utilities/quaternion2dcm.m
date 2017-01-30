function dcm = quaternion2dcm(quat)
    % QUATERNION2DCM converts a quaternion to a direction cosine matrix (DCM) representing a
    % rotation matrix.
    %
    %   INPUT ARGUMENTS:
    %       quat -- (4 x 1) quaternion vector with the scalar (real) part as first.
    %
    %   OUTPUT ARGUMENTS:
    %       dcm -- (3 x 3) rotation matrix (DCM).
    %
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017

    % notice that we are using real/imaginary serialization
    qt_sc  = quat(1);   % scalar (real) part
    qt_vec = quat(2:4); % vector (imaginary) part

    % For more background on this formula, please check
    % <https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf>, p. 101, formula (3.8).
    dcm = eye(3,3) + 2*qt_sc*skew(qt_vec) + 2*skew(qt_vec)^2;
end
