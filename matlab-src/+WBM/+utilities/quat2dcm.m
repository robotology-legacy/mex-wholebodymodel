function dcm = quat2dcm(quat)
    if (length(quat) ~= 4)
        error('quat2dcm: %s', wbmErrMsg.WRONG_VEC_SIZE);
    end

    qt_s = quat(1);   % scalar (real) part
    qt_r = quat(2:4); % vector (imaginary) part

    % Calculate the Direction Cosine Matrix (DCM) by applying the
    % Euler-Rodrigues Parameterization for efficent computing of the
    % rotation matrix R in SO(3):
    % Note: For more background on this formula, please check
    %       * https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf,
    %         page 101, formula (3.8),
    %       * or the lecture notes of Prof. Bottasso from the Aerospace
    %         Engineering Departement at the Politecnico di Milano:
    %         http://www.aero.polimi.it/~trainell/downloads/Bottasso_ThreeDimensionalRotations.pdf,
    %         page 33, formula (1.160),
    %       * or http://soliton.ae.gatech.edu/people/obauchau/publications/Bauchau+Trainelli03.pdf,
    %         page 16, formula (51).
    %       Further details about the "Rodrigues’ formula" can be found in the book
    %       "A Mathematical Introduction to Robotic Manipulation" by
    %       Murray, Li & Sastry, page 28, formula (2.14).
    dcm = eye(3) + 2*qt_s*skew(qt_r) + 2*skew(qt_r)^2;
end
