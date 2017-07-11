% function [w_c, FC_c, WC_c] = ctcwrench(f_cp, varargin)
function [w_c, wc_prms] = ctcwrench(f_cp, varargin)
    vlen = size(f_cp,1); % f_cp (force applied to the object at contact point a_p_c)
                         % must be a column-vector or a scalar.
    %% Contact wrench:
    %  Sources:
    %   [1]  GraspIt!: A Versatile Simulator for Robotic Grasping, Andrew T. Miller, PhD, Columbia University, 2001, pp. 44-50, eq. (4.1)-(4.5) & (4.10).
    %   [2]  Planning Optimal Grasps, C. Ferrari & J. Canny, Proceedings 1992 IEEE International Conference on Robotics and Automation (ICRA), 1992,
    %        <https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf>, pp. 2291-2293.
    %   [3]  From Robot to Human Grasping Simulation: 19 (Cognitive Systems Monographs), B. León & A. Morales & J. Sancho-Bru, Volume 19, Springer, 2014,
    %        pp. 16-21, eq. (2.6).
    %   [4]  A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 214-220, eq. (5.1)-(5.5).
    %   [5]  Multi-Fingered Robotic Grasping: A Primer, S. Carpin & S. Liu & J. Falco & , K. Van Wyk, University of California, Merced,
    %        2016, <https://www.nist.gov/publications/multi-fingered-robotic-grasping-primer>, pp. 6-13.
    %   [6]  Synthesis of Force-Closure Grasps on 3-D Objects Based on the Q Distance, X. Zhu & J. Wang, IEEE, 2003,
    %        <https://pdfs.semanticscholar.org/4325/e65a3cb2315134d661f4cd256526d0c992d5.pdf>, p. 670, eq. (1) & (2).
    %   [7]  Grasping Unknown Objects Based on 3D Model Reconstruction, B. Wang & L. Jiang & J. W. Li & H. G. Cai & H. Liu, Proceedings of International Conference
    %        on Advanced Intelligent Mechatronics/ASME (2005), IEEE, 2005, <http://www.cs.nott.ac.uk/~pszjl/index_files/01511025.pdf>, p. 464, eq. (2)-(6).
    %   [8]  Passivity-Based Whole-Body Balancing for Torque-Controlled Humanoid Robots in Multi-Contact Scenarios, B. Henze & M. A. Roa & C. Ott,
    %        International Journal of Robotics Research (IJRR), 2016, Volume 35, Issue 12, <http://journals.sagepub.com/doi/10.1177/0278364916653815>,
    %        p. 1525, eq. (1)-(2).
    %   [9]  Learning and Generalizing Control-Based Grasping and Manipulation Skills, Robert J. Platt, PhD, University of Massachusetts Amherst, 2006,
    %        <http://www-robotics.cs.umass.edu/uploads/Main/PlattThesis.pdf>, pp. 7-11, tbl. 2.1, eq. (2.7)-(2.11).
    %   [10] Handbook of Robotics, B. Siciliano & O. Khatib & Editors, 2nd Edition, Springer, 2016, pp. 936-938, eq. (37.9)-(37.12), p. 961, tbl. 38.4,
    %        pp. 972-973, eq. (38.57) & (38.58), fig. (38.11).
    %   [11] Shared Grasping: a Combination of Telepresence and Grasp Planning, Katharina Hertkorn, PhD, KIT Scientific Publishing, Karlsruhe, 2015,
    %        <https://www.ksp.kit.edu/9783731504023>, pp. 48-50, eq. (3.1)-(3.4).
    %   [12] On the Computation of a Common n-finger Robotic Grasp for a Set of Objects, A. Sintov & R. Menassa & A. Shapiro, Waset, International Journal of
    %        Mechanical/Industrial Science and Engineering, 2013, Volume 7, Issue 11, <http://robotics.bgu.ac.il/uploads/c/ce/RoboticGraspNov2013.PDF>,
    %        p. 2, eq. (1)-(4).
    %   [13] OCOG: A Common Grasp Computation Algorithm for a Set of Planar Objects, A. Sintov & R. Menassa & A. Shapiro, Pergamon, Robotics and
    %        Computer-Integrated Manufacturing, 2014, Volume 30, Issue 2, <https://pdfs.semanticscholar.org/36e2/e0e778b52f8ed75181c61d89db3e6f4f3051.pdf>,
    %        p. 4, eq. (1) & (2).
    %   [14] Computation of Fingertip Positions for a Form-Closure Grasp, D. Ding & Y. Liu & J. Zhang & A. Knoll, IEEE, 2001,
    %        <http://www6.in.tum.de/Main/Publications/Ding2001a.pdf>, p. 2218, eq. (1)-(4).
    switch nargin
        case 5 % contact models with friction:
            % soft-finger contact (general case):
            % a_R_c = varargin{1}, from contact frame {C} to frame {A}.
            % a_p_c = varargin{2}
            mu_s    = varargin{1,3};
            gamma_s = varargin{1,4};

            G_c = WBM.utilities.ctcmap(varargin{1,1}, varargin{1,2}, 'sfc'); % contact map
        case 4
            switch vlen
                case 4
                    % soft-finger contact (special case):
                    % a_p_c = varargin{1}
                    mu_s    = varargin{1,2};
                    gamma_s = varargin{1,3};
                    G_c = WBM.utilities.ctcmap(varargin{1,1}, 'sfc');
                case 3
                    % point contact w. friction (general case):
                    % a_R_c = varargin{1}
                    % a_p_c = varargin{2}
                    mu_s = varargin{1,3};
                    G_c = WBM.utilities.ctcmap(varargin{1,1}, varargin{1,2}, 'pcwf');
                otherwise
                    error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
        case 3
            switch vlen
                case 3
                    % point contact w. friction (special case):
                    % a_p_c = varargin{1}
                    mu_s = varargin{1,2};
                    G_c = WBM.utilities.ctcmap(varargin{1,1}, 'pcwf');
                case 1 % f_cp is a scalar (not a vector) ...
                    % frictionless contact model (general case):
                    % a_R_c = varargin{1}
                    % a_p_c = varargin{2}
                    WBM.utilities.checkValLTZero(f_cp, 'ctcwrench');

                    G_c = WBM.utilities.ctcmap(varargin{1,1}, varargin{1,2}, 'fpc');
                    w_c  = G_c * f_cp;

                    if (nargout == 2)
                        % a frictionless model has no friction cone.
                        wc_prms = struct('FC_c', [], 'Wp_c', [], 'alpha_s', 0);
                    end
                    return
                otherwise
                    error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
        case 2
            % frictionless contact model (special case):
            % a_p_c = varargin{1}
            if (vlen ~= 1)
                % f_cp is not a scalar ...
                error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            WBM.utilities.checkValLTZero(f_cp, 'ctcwrench');

            G_c = WBM.utilities.ctcmap(varargin{1,1}, 'fpc');
            w_c  = G_c * f_cp;

            if (nargout == 2)
                wc_prms = struct('FC_c', [], 'Wp_c', [], 'alpha_s', 0);
            end
            return
        otherwise
            error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % Based on the frictional contact models, calculate the contact wrench w_c and
    % the corresponding friction cone FC of the contact point a_p_c:

    % split the force vector f_cp in its components ...
    f_x = f_cp(1,1); % tangential forces
    f_y = f_cp(2,1);
    f_n = f_cp(3,1); % normal force (direction inward toward the object surface)

    if ( (f_x == 0) && (f_y == 0) )
        error('ctcwrench: Contact point force f_cp must have at least one tangential force!');
    end

    % check the friction cone constraints:
    WBM.utilities.checkValLTZero(f_n, 'ctcwrench');

    f_t = sqrt(f_x*f_x + f_y*f_y); % magnitude of the tangential contact forces.
    if (f_t > mu_s*f_n) % max. magnitude (radius) of the friction cone.
        error('ctcwrench: Contact point force f_cp has left the friction cone!');
    end
    if (vlen == 4)
        % soft-finger contact model:
        % m_z = f_cp(4)
        if (abs(f_cp(4,1)) > gamma_s*f_n) % additional FC constraint ...
            error('ctcwrench: The frictional torque m_z has exceeded the limit!');
        end
    end
    % Define the unit vectors for the approximation of the friction cone FC:
    fcp_h = f_cp./f_n; % normalize f_cp with the magnitude of f_n
                       % ("_h" ... symbol for "hat").
    %fcp_h = f_cp/sqrt(f_cp.'*f_cp); % normalization with magnitude of f_cp.
    Fcp_h = eye(3,3) .* fcp_h(1:3,1);
    % unit vectors:
    f1_h = Fcp_h(1:3,1);
    f2_h = Fcp_h(1:3,2);
    f3_h = Fcp_h(1:3,3);

    % Compute the primitive force vectors f_e(j) along
    % the edges of the m-sided polyhedron:
    if (f_x == 0) % planar cases:
        % f1_h = 0:
        m = 2;
        f_e = computePrimForcesPCone(f2_h, f3_h, mu_s);
    elseif (f_y == 0)
        % f2_h = 0:
        m = 2;
        f_e = computePrimForcesPCone(f1_h, f3_h, mu_s);
    else
        % spatial case:
        m = 8; % side number of the polyhedral cone.
        f_e = computePrimForcesPCone(f1_h, f2_h, f3_h, mu_s, m);
    end

    alpha_s = f_n/m; % scale factor (w. magnitude of f_n)
    %alpha_s = 1/m; % scale factor (w. magnitude of f_cp)

    % Calculate the primitive wrenches w_p(j) for the set of primitive contact wrenches W_p
    % and the corresponding contact wrench w_c of the contact point a_p_c:
    if (vlen == 4)
        % append the normalized mz_h to the primitive force vectors ...
        f_e = vertcat(f_e, repmat(fcp_h(4,1), 1, m));
    end
    w_p = zeros(6,m);
    w_c = zeros(6,1);
    for j = 1:m % faster than "parfor" (load time + broadcast data)
        w_p(1:6,j) = G_c * f_e(1:vlen,j);
        w_c = w_c + w_p(1:6,j);
    end
    w_c = alpha_s * w_c;
    % Note: The valid contact force f_c for grasping can be represented as a convex sum
    % (linear combination) of m force vectors f_e(j) around the boundary of the friction cone.

    if (nargout == 2)
        % Linearized friction cone FC: Set of vertices of the polyhedral cone in {C}
        % with apex. (Each matrix-column describes a vertex of the cone.)
        FC_c = horzcat(zeros(vlen,1), f_e);
        % add data ...
        wc_prms = struct('FC_c', FC_c, 'Wp_c', w_p, 'alpha_s', alpha_s);
    end
end
%% END of ctcwrench.


%% PRIMITIVE FORCES (ALONG EDGES OF PC):

function f_e = computePrimForcesPCone(varargin)
    switch nargin
        case 5
            % spatial case:
            f1_h = varargin{1,1};
            f2_h = varargin{1,2};
            f3_h = varargin{1,3};
            mu_s = varargin{1,4};
            m    = varargin{1,5};

            f_e = zeros(3,m);
            for j = 1:m
                a_p = (2*pi*j)/m;
                f_e(1:3,j) = mu_s*cos(a_p)*f1_h + mu_s*sin(a_p)*f2_h + f3_h;
            end
        case 3
            % planar case:
            ft_h = varargin{1,1};
            f3_h = varargin{1,2};
            mu_s = varargin{1,3};

            f_e = zeros(3,2);
            for j = 1:2
                a_p = (2*pi*j)/2;
                f_e(1:3,j) = mu_s*cos(a_p)*ft_h + f3_h;
            end
        otherwise
            error('computePrimForcesPCone: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
