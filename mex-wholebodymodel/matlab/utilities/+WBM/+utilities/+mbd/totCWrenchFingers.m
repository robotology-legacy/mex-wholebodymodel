function wc_tot = totCWrenchFingers(f_cp, a_R_c, a_p_c, varargin)
    %% Total contact wrench applied by the robot hand on the grasped object.
    %  This method assumes that only the fingertips are in contact with the object.
    %
    %  Sources:
    %   [1] Synthesis of Force-Closure Grasps on 3-D Objects Based on the Q Distance, X. Zhu & J. Wang, IEEE, 2003,
    %       <https://pdfs.semanticscholar.org/4325/e65a3cb2315134d661f4cd256526d0c992d5.pdf>, p. 670, eq. (3).
    %   [2] Grasp quality measures: review and performance, M. A. Roa & R. Suárez, Springer, Autonomous Robots, 2015, Volume 38, Issue 1,
    %       <https://link.springer.com/article/10.1007/s10514-014-9402-3>, p. 71, eq. (20) & (22).
    %   [3] Planning Optimal Grasps, C. Ferrari & J. Canny, IEEE International Conference on Robotics and Automation (ICRA), 1992,
    %       <https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf>, p. 2293.
    %   [4] Grasping Unknown Objects Based on 3D Model Reconstruction, B. Wang & L. Jiang & J. W. Li & H. G. Cai & H. Liu, Proceedings of International Conference
    %       on Advanced Intelligent Mechatronics/ASME (2005), IEEE, 2005, <http://www.cs.nott.ac.uk/~pszjl/index_files/01511025.pdf>, p. 464, eq. (7).
    wc_tot = zeros(6,1);
    n_c    = size(f_cp,2); % number of contacts (used fingers) -- f_cp must be a row
                           % cell-array with column-vectors or scalars as elements.
    if (n_c >= 1)
        switch nargin
            case 6
                % soft-finger contact:
                % mu_s    = varargin{1}
                % gamma_s = varargin{2}
                for i = 1:n_c
                    w_c    = WBM.utilities.mbd.ctcwrench(f_cp{1,i}, a_R_c, a_p_c, varargin{1:2});
                    wc_tot = wc_tot + w_c;
                end
            case 5
                % point contact w. friction:
                % mu_s = varargin{1}
                for i = 1:n_c
                    w_c    = WBM.utilities.mbd.ctcwrench(f_cp{1,i}, a_R_c, a_p_c, varargin{1,1});
                    wc_tot = wc_tot + w_c;
                end
            case 4
                % frictionless contact model:
                for i = 1:n_c
                    w_c    = WBM.utilities.mbd.ctcwrench(f_cp{1,i}, a_R_c, a_p_c);
                    wc_tot = wc_tot + w_c;
                end
            otherwise
                error('totCWrenchFingers: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
        end
    end
end
