function X_v = mixvelcp(varargin)
    %% Spatial cross product operator for a
    %  mixed velocity vector (v, w)^T in R^6:
    %
    % Sources:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, pp. 8-9, eq. (38).
    %   [2] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, pp. 23-25, eq. (2.32).
    switch nargin
        case 2
            v = varargin{1,1}; % linear velocity
            w = varargin{1,2}; % angular velocity
        case 1
            v_m = varargin{1,1};
            WBM.utilities.chkfun.checkCVecDim(v_m, 6, 'mixvelcp');

            % get the linear and angular velocity
            % of the mixed velocity v_m ...
            v = v_m(1:3,1);
            w = v_m(4:6,1);
        otherwise
            error('mixvelcp: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % skew-symmetric matrices of the velocities ...
    Sv = WBM.utilities.tfms.skewm(v);
    Sw = WBM.utilities.tfms.skewm(w);

    % create the spatial cross operator:
    X_v = zeros(6,6);
    X_v(1:3,1:3) = Sw;
    X_v(1:3,4:6) = Sv;
    X_v(4:6,4:6) = Sw;
end
