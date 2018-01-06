function G_c = cmap(varargin)
    %% Contact map:
    %  Sources:
    %   [1] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 218-220.
    %   [2] Multi-Fingered Robotic Grasping: A Primer, S. Carpin & S. Liu & J. Falco & , K. Van Wyk, University of California, Merced,
    %       2016, <https://www.nist.gov/publications/multi-fingered-robotic-grasping-primer>, pp. 7-9.
    %   [3] GraspIt!: A Versatile Simulator for Robotic Grasping, Andrew T. Miller, PhD, Columbia University, 2001, p. 47, eq. (4.5).
    %   [4] Learning and Generalizing Control-Based Grasping and Manipulation Skills, Robert J. Platt, PhD, University of Massachusetts Amherst, 2006,
    %       <http://www-robotics.cs.umass.edu/uploads/Main/PlattThesis.pdf>, p. 11, eq. 2.11.
    %   [5] Passivity-Based Whole-Body Balancing for Torque-Controlled Humanoid Eobots in Multi-Contact Scenarios, B. Henze & M. A. Roa & C. Ott,
    %       International Journal of Robotics Research (IJRR), 2016, Volume 35, Issue 12, <http://journals.sagepub.com/doi/abs/10.1177/0278364916653815?journalCode=ijra>,
    %       p. 1525, eq. (3).
    switch nargin
        case 3
            % general case:
            o_R_c = varargin{1,1};
            o_p_c = varargin{1,2};
            % ctc_type = varargin{3}

            WBM.utilities.chkfun.checkMatDim(o_R_c, 3, 3, 'cmap');
            WBM.utilities.chkfun.checkCVecDim(o_p_c, 3, 'cmap');

            o_cX_c = WBM.utilities.tfms.coadjoint(o_R_c, o_p_c);
            B_c    = WBM.utilities.mbd.wbasis(varargin{1,3});
            G_c    = o_cX_c * B_c;
        case 2
            % special case:
            % The rotation matrix o_R_c (from the contact frame {C} to the origin frame {O}
            % at the CoM of the object) is an identity matrix and o_p_c is not zero.
            o_p_c    = varargin{1,1};
            ctc_type = varargin{1,2};

            WBM.utilities.chkfun.checkCVecDim(o_p_c, 3, 'cmap');

            switch ctc_type
                case 'pcwf'
                    % point contact with friction:
                    G_c = zeros(6,3);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.tfms.skewm(o_p_c);
                case 'sfc'
                    % soft-finger contact (with friction):
                    G_c = zeros(6,4);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.tfms.skewm(o_p_c);
                    G_c(6,4)     = 1;
                case 'fpc'
                    % frictionless point contact:
                    % (rarely used, i.e. if the friction is
                    % negligibly low or unknown)
                    G_c = zeros(6,1);
                    G_c(3,1) =  1;
                    G_c(4,1) =  o_p_c(2,1);
                    G_c(5,1) = -o_p_c(1,1);
                otherwise
                    error('cmap: %s', WBM.wbmErrorMsg.UNKNOWN_CTC_MODEL);
            end
        otherwise
            error('cmap: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
