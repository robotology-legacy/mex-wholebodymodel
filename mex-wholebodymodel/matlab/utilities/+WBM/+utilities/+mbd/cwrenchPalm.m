function wc_tot = cwrenchPalm(f_cp, o_R_c, o_p_c, varargin)
    % CWRENCHPALM computes the total contact wrench of the palm
    % of the robot hand. This simple method is useful if a rigid
    % body with a point mass (at the CoM) will be grasped only
    % by the palms of the robot hands.
    %
    %   INPUT ARGUMENTS:
    %       f_cp    -- force applied to the object at contact point o_p_c in contact frame {C} (must be a vector or a scalar)
    %       o_R_c   -- (3 x 3) rotation matrix from the contact frame {C} to the origin frame {O} at the CoM of the object
    %       o_p_c   -- (3 x 1) contact point vector from the contact frame {C} to the origin frame {O} at the CoM of the object
    %       mu_s    -- static friction coefficient for surfaces (optional)
    %       gamma_s -- static torsional friction coefficient for surfaces (optional)
    %
    %   OUTPUT ARGUMENTS:
    %       wc_tot -- (6 x 1) total contact wrench vector in frame {C} (contact space).
    %
    switch nargin
        case 5
            % soft-finger contact:
            % mu_s    = varargin{1}
            % gamma_s = varargin{2}
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, o_R_c, o_p_c, varargin{1:2});
        case 4
            % point contact w. friction:
            % mu_s = varargin{1}
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, o_R_c, o_p_c, varargin{1,1});
        case 3
            % frictionless contact model:
            wc_tot = WBM.utilities.mbd.cwrench(f_cp, o_R_c, o_p_c);
        otherwise
            error('cwrenchPalm: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
