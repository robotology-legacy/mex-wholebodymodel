function G_c = ctcmap(varargin)
    %% Contact map:
    %  Sources:
    %   [1] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 218-220.
    %   [2] Multi-Fingered Robotic Grasping: A Primer, S. Carpin & S. Liu & J. Falco & , K. Van Wyk, University of California, Merced,
    %       2016, <https://www.nist.gov/publications/multi-fingered-robotic-grasping-primer>, pp. 7-9.
    %   [3] GraspIt!: A Versatile Simulator for Robotic Grasping, Andrew T. Miller, PhD, Columbia University, 2001, p. 47, eq. (4.5).
    switch nargin
        case 3
            % general case:
            % wf_R_c   = varargin{1}
            % ctc_type = varargin{3}
            r_c = varargin{1,2};

            WBM.utilities.checkCVecDim(r_c, 3, 'ctcmap');

            wf_cX_c = WBM.utilities.coadjoint(varargin{1,1}, r_c);
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
                case 'pt_ctc'
                    % frictionless point contact:
                    G_c = zeros(6,1);
                    G_c(3,1) =  1;
                    G_c(4,1) =  r_c(2,1);
                    G_c(5,1) = -r_c(1,1);
                case 'hard_ctc'
                    % point contact with friction:
                    G_c = zeros(6,3);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.skewm(r_c);
                case 'soft_ctc'
                    % soft-finger contact (with friction):
                    G_c = zeros(6,4);
                    G_c(1:3,1:3) = eye(3,3);
                    G_c(4:6,1:3) = WBM.utilities.skewm(r_c);
                    G_c(6,4)     = 1;
                otherwise
                    error('ctcmap: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        otherwise
            error('ctcmap: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
