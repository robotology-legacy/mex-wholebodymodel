function B_c = wbasis(ctc_type)
    %% Wrench basis:
    %  Sources:
    %   [1] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 214-219, tbl. 5.2, eq. (5.2), (5.3) & (5.4).
    %   [2] Learning and Generalizing Control-Based Grasping and Manipulation Skills, Robert J. Platt, PhD, University of Massachusetts Amherst, 2006,
    %       <http://www-robotics.cs.umass.edu/uploads/Main/PlattThesis.pdf>, p. 9, tbl. 2.1.
    %   [3] Handbook of Robotics, B. Siciliano & O. Khatib & Editors, 2nd Edition, Springer, 2016, p. 938, eq. (37.10), (37.11) & (37.12).
    switch ctc_type
        case 'pt_ctc'
            % frictionless point contact:
            B_c = zeros(6,1);
            B_c(3,1) = 1;
        case 'hard_ctc'
            % point contact with friction:
            B_c = zeros(6,3);
            B_c(1:3,1:3) = eye(3,3);
        case 'soft_ctc'
            % soft-finger contact (with friction):
            B_c = zeros(6,4);
            B_c(1:3,1:3) = eye(3,3);
            B_c(6,4)     = 1;
        otherwise
            error('wbasis: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
end
