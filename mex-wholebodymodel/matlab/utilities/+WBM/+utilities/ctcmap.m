function G_c = ctcmap(varargin)
    %% Contact map:
    %  Sources:
    %   [1] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 218-220.
    %   [2] Multi-Fingered Robotic Grasping: A Primer, S. Carpin & S. Liu & J. Falco & , K. Van Wyk, University of California, Merced,
    %       2016, <https://www.nist.gov/publications/multi-fingered-robotic-grasping-primer>, pp. 7-9.
    %   [3] GraspIt!: A Versatile Simulator for Robotic Grasping, Andrew T. Miller, PhD, Columbia University, 2001, p. 47, eq. (4.5).
    %   [4] Learning and Generalizing Control-Based Grasping and Manipulation Skills, Robert J. Platt, PhD, University of Massachusetts Amherst, 2006,
    %       <http://www-robotics.cs.umass.edu/uploads/Main/PlattThesis.pdf>, p. 11, eq. 2.11.
    %   [5] Passivity-Based Whole-Body Balancing for Torque-Controlled Humanoid Eobots in Multi-Contact Scenarios, B. Henze & M. A. Roa & C. Ott,
    %       Vol 35, Issue 12, 2016, The International Journal of Robotics Research (IJRR), <http://journals.sagepub.com/doi/abs/10.1177/0278364916653815?journalCode=ijra>,
    %       p. 1525, eq. (3).
    switch nargin
        case 3
            % general case:
            wf_R_c = varargin{1,1};
            r_c    = varargin{1,2};
            % ctc_type = varargin{3}

            WBM.utilities.checkMatDim(wf_R_c, 3, 3, 'ctcmap');
            WBM.utilities.checkCVecDim(r_c, 3, 'ctcmap');

            wf_cX_c = WBM.utilities.coadjoint(wf_R_c, r_c);
            B_c     = WBM.utilities.wbasis(varargin{1,3});
            G_c     = wf_cX_c * B_c;
        case 2
            % special case:
            % The rotation matrix wf_R_c (from WF to contact frame {C})
            % is an identity matrix and r_c is not zero.
            r_c      = varargin{1,1};
            ctc_type = varargin{1,2};

            WBM.utilities.checkCVecDim(r_c, 3, 'ctcmap');

            switch ctc_type
                case 'pcwf'
                    % point contact with friction:
                    G_c = zeros(6,3);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.skewm(r_c);
                case 'sfc'
                    % soft-finger contact (with friction):
                    G_c = zeros(6,4);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.skewm(r_c);
                    G_c(6,4)     = 1;
                case 'fpc'
                    % frictionless point contact:
                    % (rarely used, i.e. if the friction is
                    % negligibly low or unknown)
                    G_c = zeros(6,1);
                    G_c(3,1) =  1;
                    G_c(4,1) =  r_c(2,1);
                    G_c(5,1) = -r_c(1,1);
                otherwise
                    error('ctcmap: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        otherwise
            error('ctcmap: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
