function wc_tot = cwrenchPalm(f_cp, a_R_c, a_p_c, varargin)
    %% Total contact wrench of the palm of the robot hand.
    %  This simple method is useful if a rigid body with a point
    %  mass will be grasped only by the palms of the robot hands.
    switch nargin
        case 5
            % soft-finger contact:
            % mu_s    = varargin{1}
            % gamma_s = varargin{2}
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, a_R_c, a_p_c, varargin{1:2});
        case 4
            % point contact w. friction:
            % mu_s = varargin{1}
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, a_R_c, a_p_c, varargin{1,1});
        case 3
            % frictionless contact model:
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, a_R_c, a_p_c);
        otherwise
            error('cwrenchPalm: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
