function M_rb = generalizedInertia(m_rb, I_cm, varargin)
    %% Generalized inertia tensor of a rigid body:
    %  {C} is an orientation frame of the body's center of mass (CoM) and
    %  {A} is a generic frame defined according to
    %
    %       A_(M_rb)_A = A_cX_C * C_(M_rb)_C * C_X_A.
    %
    %  Sources:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 12, eq. (58)-(60).
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 162-163, eq. (4.9).
    %   [3] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, pp. 32-33, eq. (2.61)-(2.63).
    %   [4] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, pp. 13-14, eq. (2.9) & (2.11).
    %   [5] Informatics in Control, Automation and Robotics, J. A. Cetto & J. Ferrier & J. Filipe,
    %       Lecture Notes in Electrical Engineering, Volume 89, Springer, 2011, p. 7, eq. (9).
    %   [6] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
    %       Springer, 2010, p. 582, eq. (B.11) & (B.12).
    switch nargin
        case 4
            % general case with respect to a generic frame {A}:
            a_R_cm = varargin{1,1};
            a_p_cm = varargin{1,2};

            % applying the parallel-axis theorem (Huygens-Steiner theorem):
            Sp   = WBM.utilities.tfms.skewm(a_p_cm);
            Sp2  = Sp * Sp;
            cm_R_a = a_R_cm.';

            I_a = a_R_cm * I_cm * cm_R_a - m_rb * Sp2; %  a_R_cm = cm_R_a^T,
                                                       % -S(a_p_cm) = S(a_p_cm)^T, S ... skewm
        case 3 % special cases:
            % with respect to a frame {C} centered at a_p_cm:
            % (if a_R_cm is an identity matrix and a_p_cm is not zero)
            a_p_cm = varargin{1,1};

            % applying the parallel-axis theorem:
            Sp  = WBM.utilities.tfms.skewm(a_p_cm);
            Sp2 = Sp * Sp;
            I_a = I_cm - m_rb * Sp2; % a_R_cm = cm_R_a = eye(3,3)
        case 2
            % with respect to a frame {C} centered at the body's CoM:
            % (if wf_R_cm is an identity matrix and wf_p_cm is a 0-vector)
            M_rb = zeros(6,6);
            M_rb(1:3,1:3) = m_rb * eye(3,3);
            M_rb(4:6,4:6) = I_cm;
            return
        otherwise
            error('generalizedInertia: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % create the generalized inertia matrix ...
    h = m_rb * Sp;

    M_rb = zeros(6,6);
    M_rb(1:3,1:3) =  m_rb * eye(3,3);
    M_rb(4:6,1:3) =  h;
    M_rb(1:3,4:6) = -h; % = h^T
    M_rb(4:6,4:6) =  I_a;
end
