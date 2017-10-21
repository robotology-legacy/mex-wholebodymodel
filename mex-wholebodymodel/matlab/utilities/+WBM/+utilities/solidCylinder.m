function [X, Y, Z] = solidCylinder(varargin)
    ri0 = 0; % inner radius (zero)
    hax = [];
    switch nargin
        case 3
            % hax = varargin{1} ... axes handle to plot into a specified axes
            % r   = varargin{2} ... radius vector of the (linearized) cylinder
            % n   = varargin{3} ... number of n equally spaced points around the cylinder
            [xx, yy, zz] = WBM.utilities.solidRing(varargin{1,1}, ri0, varargin{1,2:3}, 'rect');
        case 2
            if isscalar(varargin{1,1})
                % r = varargin{1}
                % n = varargin{2}
                [xx, yy, zz] = WBM.utilities.solidRing(ri0, varargin{1,1:2}, 'rect');
            else
                % hax = varargin{1}
                % r   = varargin{2}
                [xx, yy, zz] = WBM.utilities.solidRing(varargin{1,1}, ri0, varargin{1,2}, 'rect');
            end
        case 1
            % r = varargin{1}
            [xx, yy, zz] = WBM.utilities.solidRing(ri0, varargin{1,1}, 'rect');
        case 0
            % use the unit radius (default)
            [xx, yy, zz] = WBM.utilities.solidRing(ri0, 1, 'rect');
    end

    if (nargout == 0)
        hax = newplot(hax);
        surf(xx, yy, zz, 'Parent', hax);
    else
        X = xx; Y = yy; Z = zz;
    end
end
