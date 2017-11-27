function [X, Y, Z] = solidRing(varargin)
    % Adapted and extended version of the given source code excerpts at
    % <https://stackoverflow.com/questions/27668343/3d-ring-in-matlab-using-patch>

    % default values:
    n       = 20; % number of vertices of the linearized ring
    hax     = [];
    cs_type = 'rect';
    cs.rect = true;
    switch nargin % process input arguments ...
        case 5
            hax     = varargin{1,1}; % axes handle to plot into a specified axes
            ri      = varargin{1,2}; % inner radius of the (linearized) ring
            ro      = varargin{1,3}; % outer radius of the (linearized) ring
            n       = varargin{1,4}; % number of n equally spaced points around the ring
            cs_type = varargin{1,5}; % cross section type for the ring (circular or rectangular)

            checkAxesHandle(hax);
            checkRadiusVals(ri, ro);
            checkVertexVal(n);
        case 4
            cs_type = varargin{1,4};

            if isscalar(varargin{1,1})
                ri = varargin{1,1};
                ro = varargin{1,2};
                n  = varargin{1,3};

                checkVertexVal(n);
            else
                hax = varargin{1,1};
                ri  = varargin{1,2};
                ro  = varargin{1,3};

                checkAxesHandle(hax);
            end
            checkRadiusVals(ri, ro);
        case 3
            if ischar(varargin{1,3})
                ri      = varargin{1,1};
                ro      = varargin{1,2};
                cs_type = varargin{1,3};
            elseif isscalar(varargin{1,1})
                ri = varargin{1,1};
                ro = varargin{1,2};
                n  = varargin{1,3};

                checkVertexVal(n);
            else
                hax = varargin{1,1};
                ri  = varargin{1,2};
                ro  = varargin{1,3};

                checkAxesHandle(hax);
            end
            checkRadiusVals(ri, ro);
        case 2
            ri = varargin{1,1};
            ro = varargin{1,2};

            checkRadiusVals(ri, ro);
        case 0
            % use the default values:
            ri = 0.5;
            ro = 1; % unit radius
    end
    nvtx = n + 1; % n vertices (bottom) with vertex n+1 (top). nvtx_tot = 2*n.
    phi = linspace(0, 2*pi, nvtx); % angles between the cross sections (CS)
    ncs = size(phi,2); % number of cross sections

    %% Create the cross section profile (CSP) in the xz-plane:
    switch cs_type
        case 'rect'
            % rectangular CSP with unit height 1:
            npts     = 4;
            cs.y0    = zeros(1,npts); % base y-positions for rotating the cross sections
            cs.wid_h = (ro - ri)*0.5; % half width of each cross section to the center
            cs.hgt_h = 0.5;           % half height of each cross section to the center

            ptx_o =  cs.wid_h; % inner x-pos.
            ptx_i = -cs.wid_h; % outer x-pos.

            ptz_t =  cs.hgt_h; % z-pos. top
            ptz_b = -cs.hgt_h; % z-pos. bottom

            cs.x = horzcat(ptx_i, ptx_o, ptx_o, ptx_i);
            cs.z = horzcat(ptz_b, ptz_b, ptz_t, ptz_t);
        case 'circ'
            % circular CSP with unit diameter 1:
            cs.rect = false;
            cs.phi  = linspace(0, 2*pi, ncs);
            npts    = size(cs.phi,2);

            cs.y0 = zeros(1,npts);
            cs.r  = (ro - ri)*0.5;
            cs.x  = cs.r * sin(cs.phi);
            cs.z  = cs.r * cos(cs.phi);
            cs.x(1,npts) = 0; % value correction: overwrite -0.0000
        otherwise
            error('solidRing: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
    % transform the polar coordinates of the vertex nodes
    % of the core circle to Cartesian:
    rc = (ri + ro)*0.5; % radius of the core circle (= ri + (ro - ri)/2)
    ring.x = rc * cos(phi);
    ring.y = rc * sin(phi);
    ring.y(1,nvtx) = 0; % correction

    %% Generate the coordinates of each cross section (CS):
    % initialization ...
    pos_xy0 =  vertcat(cs.x, cs.y0);
    rotm11  =  cos(phi(1:ncs));
    rotm21  =  sin(phi(1:ncs));
    rotm12  = -rotm21;
    rotm22  =  rotm11;
    rotm    =  zeros(2,2);

    xx = zeros(ncs,npts); yy = xx; zz = xx;
    for i = 1:ncs
        % rotate the current CS around
        % the z-axis (origin) ...
        rotm(1,1) = rotm11(1,i);
        rotm(1,2) = rotm12(1,i);
        rotm(2,1) = rotm21(1,i);
        rotm(2,2) = rotm22(1,i);
        cs_t = rotm * pos_xy0;

        % translate the coordinates of the CS to
        % the position of the core circle:
        xx(i,:) = cs_t(1,:) + ring.x(1,i);
        yy(i,:) = cs_t(2,:) + ring.y(1,i);
        zz(i,:) = cs.z;
    end

    if ( cs.rect && (ri > 0) )
        % add the coordinates for the inner cylinder ...
        xx = horzcat(xx(:,1), xx, xx(:,4));
        yy = horzcat(yy(:,1), yy, yy(:,4));
        zz = horzcat(zz(:,4), zz, zz(:,1));
    end

    if (nargout == 0)
        hax = newplot(hax);
        surf(xx, yy, zz, 'Parent', hax);
    else
        X = xx; Y = yy; Z = zz;
    end
end
%% END of solidRing.


%% CHECK FUNCTIONS:

function checkVertexVal(n)
    assert(isnumeric(n) && isscalar(n) && isfinite(n) && ...
           (imag(n) == 0) && (n > 2) && (round(n) == n), ...
           'solidRing: Argument ''n'' must be a finite, real, scalar integer greater than 2.');
end

function checkRadiusVals(ri, ro)
    assert(isnumeric(ri) && isscalar(ri) && isfinite(ri) && (imag(ri) == 0) && (ri >= 0), ...
           'solidRing: Argument ''ri'' must be a finite, postive, real, scalar integer.');
    assert(isnumeric(ro) && isscalar(ro) && isfinite(ro) && (imag(ro) == 0) && (ro >= 0), ...
           'solidRing: Argument ''ro'' must be a finite, postive, real, scalar integer.');
    assert(((ro - ri) >= 0), 'solidRing: Argument ''ri'' must be smaller or equal to ''ro''.');
end

function checkAxesHandle(hax)
    assert(ishandle(hax) && strcmp(get(hax, 'type'), 'axes'), ...
           'solidRing: Argument ''hax'' must be a valid axis handle.');
end
