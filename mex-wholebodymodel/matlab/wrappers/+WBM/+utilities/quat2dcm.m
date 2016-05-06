function dcm = quat2dcm(quat)
    if (length(quat) ~= 4)
        error('quat2dcm: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
    end

    qt_s = quat(1);   % scalar (real) part
    qt_r = quat(2:4); % vector (imaginary) part

    % Calculate the Direction Cosine Matrix (DCM) by applying the
    % Euler-Rodrigues Parameterization for efficent computing of the
    % rotation matrix R in SO(3):
    % Note: For more background on this formula, please check
    %       [1] CONTRIBUTIONS TO THE AUTOMATIC CONTROL OF AERIAL VEHICLES, Minh Duc HUA, PhD-Thesis, 2009,
    %           p. 101, formula (3.8), <https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf>
    %       [2] Lecture notes for 'Multibody System Dynamics', Prof. Bottasso, Aerospace Engineering Departement, Politecnico di Milano,
    %           p. 33, formula (1.160), <https://home.aero.polimi.it/trainelli/downloads/Bottasso_ThreeDimensionalRotations.pdf>
    %       [3] The Vectorial Parameterization of Rotation, Olivier A. Bauchau & Lorenzo Trainelli, Nonlinear Dynamics, 2003,
    %           p. 16, formula (51), <http://soliton.ae.gatech.edu/people/obauchau/publications/Bauchau+Trainelli03.pdf>
    %       Further details about the "Rodrigues’ formula" can be found in the book,
    %       [4] A Mathematical Introduction to Robotic Manipulation, Murray, Li & Sastry, CRC Press, 1994, p. 28, formula (2.14).
    dcm = eye(3) + 2*qt_s*skew(qt_r) + 2*skew(qt_r)^2;
end
