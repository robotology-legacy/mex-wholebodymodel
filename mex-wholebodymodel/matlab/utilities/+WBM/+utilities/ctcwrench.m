function [w_c, FC_c] = ctcwrench(f_cp, varargin)
    vlen = size(f_cp,1); % f_cp must be a column-vector or a scalar.

    %% Contact wrench:
    %  Sources:
    %   [1]  GraspIt!: A Versatile Simulator for Robotic Grasping, Andrew T. Miller, PhD, Columbia University, 2001, pp. 44-50, eq. (4.1)-(4.5) & (4.10).
    %   [2]  Planning Optimal Grasps, C. Ferrari & J. Canny, Proceedings 1992 IEEE International Conference on Robotics and Automation (ICRA), 1992,
    %        <https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf>, pp. 2291-2293.
    %   [3]  From Robot to Human Grasping Simulation: 19 (Cognitive Systems Monographs), B. León & A. Morales & J. Sancho-Bru, Volume 19, Springer, 2014,
    %        p. 18-21, eq. (2.6).
    %   [4]  A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 214-218, eq. (5.1)-(5.5).
    %   [5]  Multi-Fingered Robotic Grasping: A Primer, S. Carpin & S. Liu & J. Falco & , K. Van Wyk, University of California, Merced,
    %        2016, <https://www.nist.gov/publications/multi-fingered-robotic-grasping-primer>, pp. 6-12.
    %   [6]  Synthesis of Force-Closure Grasps on 3-D Objects Based on the Q Distance, X. Zhu & J. Wang, IEEE, 2003,
    %        <https://pdfs.semanticscholar.org/4325/e65a3cb2315134d661f4cd256526d0c992d5.pdf>, p. 670, eq. (1).
    %   [7]  Grasping Unknown Objects Based on 3D Model Reconstruction, B. Wang & L. Jiang & J. W. Li & H. G. Cai & H. Liu, Proceedings of International Conference
    %        on Advanced Intelligent Mechatronics/ASME (2005), IEEE, 2005, <http://www.cs.nott.ac.uk/~pszjl/index_files/01511025.pdf>, p. 464, eq. (2)-(5).
    %   [8]  Passivity-Based Whole-Body Balancing for Torque-Controlled Humanoid Robots in Multi-Contact Scenarios, B. Henze & M. A. Roa & C. Ott,
    %        International Journal of Robotics Research (IJRR), 2016, Volume 35, Issue 12, <http://journals.sagepub.com/doi/10.1177/0278364916653815>,
    %        p. 1525, eq. (1)-(3).
    %   [9]  Learning and Generalizing Control-Based Grasping and Manipulation Skills, Robert J. Platt, PhD, University of Massachusetts Amherst, 2006,
    %        <http://www-robotics.cs.umass.edu/uploads/Main/PlattThesis.pdf>, pp. 7-11, tbl. 2.1, eq. (2.7)-(2.11).
    %   [10] Handbook of Robotics, B. Siciliano & O. Khatib & Editors, 2nd Edition, Springer, 2016, pp. 936-938, eq. (37.9)-(37.12), p. 961, tbl. 38.4,
    %        pp. 972-973, eq. (38.57) & (38.58).
    %   [11] Shared Grasping: a Combination of Telepresence and Grasp Planning, Katharina Hertkorn, PhD, KIT Scientific Publishing, Karlsruhe, 2015,
    %        <https://www.ksp.kit.edu/9783731504023>, pp. 48-50, eq. (3.1)-(3.4).
    %   [12] On the Computation of a Common n-finger Robotic Grasp for a Set of Objects, A. Sintov & R. Menassa & A. Shapiro, Waset, International Journal of
    %        Mechanical/Industrial Science and Engineering, 2013, Volume 7, Issue 11, <http://robotics.bgu.ac.il/uploads/c/ce/RoboticGraspNov2013.PDF>,
    %        p. 2, eq. (1)-(4).
    %   [13] OCOG: A Common Grasp Computation Algorithm for a Set of Planar Objects, A. Sintov & R. Menassa & A. Shapiro, Pergamon, Robotics and
    %        Computer-Integrated Manufacturing, 2014, Volume 30, Issue 2, <https://pdfs.semanticscholar.org/36e2/e0e778b52f8ed75181c61d89db3e6f4f3051.pdf>,
    %        p. 4, eq. (1) & (2).
    switch nargin
        case 5 % contact models with friction:
            % soft-finger contact (general case):
            mu_s    = varargin{1,1};
            gamma_s = varargin{1,2};
            % wf_R_c = varargin{3}
            % r_c    = varargin{4}

            G_c = WBM.utilities.ctcmap(varargin{1,3}, varargin{1,4}, 'sfc'); % contact map
        case 4
            mu_s = varargin{1,1};
            r_c  = varargin{1,3};

            switch vlen
                case 4
                    % soft-finger contact (special case):
                    gamma_s = varargin{1,2};
                    G_c = WBM.utilities.ctcmap(r_c, 'sfc');
                case 3
                    % point contact w. friction (general case):
                    % wf_R_c = varargin{2}
                    G_c = WBM.utilities.ctcmap(varargin{1,2}, r_c, 'pcwf');
                otherwise
                    error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
        case 3
            r_c = varargin{1,2};

            switch vlen
                case 3
                    % point contact w. friction (special case):
                    mu_s = varargin{1,1};
                    G_c = WBM.utilities.ctcmap(r_c, 'pcwf');
                case 1 % f_cp is a scalar (not a vector) ...
                    % frictionless contact model (general case):
                    % wf_R_c = varargin{1}
                    checkLTzero(f_cp, 'ctcwrench');

                    G_c = WBM.utilities.ctcmap(varargin{1,1}, r_c, 'fpc');
                    w_c  = G_c * f_cp;
                    FC_c = []; % a frictionless model has no friction cone.
                    return
                otherwise
                    error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
        case 2
            % frictionless contact model (special case):
            % r_c = varargin{1}
            if (vlen ~= 1)
                % f_cp is not a scalar ...
                error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            checkLTzero(f_cp, 'ctcwrench');

            G_c = WBM.utilities.ctcmap(varargin{1,1}, 'fpc');
            w_c  = G_c * f_cp;
            FC_c = [];
            return
        otherwise
            error('ctcwrench: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % Calculate for the the frictional contact models the contact wrench w_c
    % and the corresponding friction cone FC at the current contact point c_i
    % in frame {C_i}:

    % break the force vector f_cp in its components ...
    f_x  = f_cp(1,1); % tangential forces
    f_y  = f_cp(2,1);
    f_n  = f_cp(3,1); % normal force (direction inward toward the object surface)

    % ft_c = mu_s*f_n; % max. magnitude (radius) of the friction cone.
    if ( (f_x == 0) && (f_y == 0) )
        error('ctcwrench: Contact point force f_cp must have at least one tangential force!');
    end

    % check the friction cone constraints:
    checkLTzero(f_n, 'ctcwrench');

    f_t = sqrt(f_x*f_x + f_y*f_y); % magnitude of the tangential contact forces.
    if (f_t > mu_s*f_n) % max. magnitude (radius) of the friction cone.
        error('ctcwrench: Contact point force f_cp has left the friction cone!');
    end
    if (vlen == 4)
        % soft-finger contact model:
        m_z = f_cp(4,1);

        % additional FC constraint ...
        if (abs(m_z) > gamma_s*f_n)
            error('ctcwrench: The frictional torque m_z has exceeded the limit!');
        end
    end
    % define the unit vectors for the approximation of the friction cone FC:
    fcp_h = f_cp./f_n; % normalize f_cp with the magnitude of f_n.
    %fcp_h = f_cp/sqrt(f_cp.'*f_cp); % normalization with magnitude of f_cp.

    Fcp_h = eye(3,3).*fcp_h;
    % unit vectors:
    f1_h = Fcp_h(1:3,1);
    f2_h = Fcp_h(1:3,2);
    f3_h = Fcp_h(1:3,3);

    % compute the force vectors f_e along the edges of the m-sided polyhedron:
    if (f_x == 0) % planar cases:
        % f1_h = 0:
        m = 2;
        f_e = computePConeEdgeForces(f2_h, f3_h, mu_s);
    elseif (f_y == 0)
        % f2_h = 0:
        m = 2;
        f_e = computePConeEdgeForces(f1_h, f3_h, mu_s);
    else
        % spatial case:
        m = 8; % side number of the polyhedral cone.
        f_e = computePConeEdgeForces(f1_h, f2_h, f3_h, mu_s, m);
    end

    % the valid contact force f_c for grasping can be represented as a convex
    % sum of m force vectors around the boundary of the friction cone:
    a_s = f_n/m; % scale factor (w. magnitude of f_n)
    %a_s = 1/m; % scale factor (w. magnitude of f_cp)

    f_c = zeros(3,1);
    for j = 1:m
        f_c = f_c + f_e(1:3,j);
    end
    f_c = a_s * f_c;

    % vertices of the polyhedral cone in {C_i}:
    FC_c = horzcat(zeros(3,1), f_e);
    % contact wrench:
    if (vlen == 4)
        f_c = vertcat(f_c, m_z);
    end
    w_c = G_c * f_c;
end
%% END of ctcwrench.


%% CHECK FUNCTION & EDGE FORCES:

function f_e = computePConeEdgeForces(varargin)
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
            error('computePConeEdgeForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end

function checkLTzero(val, func_name)
    if (val < 0)
        error('%s: %s', func_name, WBM.wbmErrorMsg.VALUE_LTE_ZERO);
    end
end
