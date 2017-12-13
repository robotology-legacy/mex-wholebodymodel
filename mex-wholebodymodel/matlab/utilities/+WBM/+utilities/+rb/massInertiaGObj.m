function [m_rb, I_cm] = massInertiaGObj(obj_type, rho, varargin)
    %% Returns the mass and the corresponding inertia tensor of a geometric object with
    %  the origin of the coordinate system at the object body's center of mass (CoM):
    %
    %  Sources:
    %   [1] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 163.
    %   [2] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
    %       pp. 169-171, eq. (6.28).
    %   [3] Eric Weisstein's World of Physics: Moment of Inertia--Cylinder, Eric W. Weisstein, scienceworld.wolfram.com, 1996-2007,
    %       <http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html>
    %   [4] Eric Weisstein's World of Physics: Moment of Inertia--Sphere, Eric W. Weisstein, scienceworld.wolfram.com, 1996-2007,
    %       <http://scienceworld.wolfram.com/physics/MomentofInertiaSphere.html>
    %   [5] Eric Weisstein's World of Physics: Moment of Inertia--Spherical Shell, Eric W. Weisstein, scienceworld.wolfram.com, 1996-2007,
    %       <http://scienceworld.wolfram.com/physics/MomentofInertiaSphericalShell.html>
    %   [6] List of moments of inertia: List of 3D inertia tensors, Wikipedia, 2017, <https://en.wikipedia.org/wiki/List_of_moments_of_inertia>
    switch obj_type
        case 'scub'
            % solid (rectangular) cuboid:
            switch nargin
                case 5
                    % cuboid:
                    l = varargin{1,1}; % length (in y-direction)
                    w = varargin{1,2}; % width (in x-direction)
                    h = varargin{1,3}; % height (in z-direction)

                    m_rb = rho*(l*w*h); % mass of the cuboid
                    m12  = m_rb/12;

                    I_xx = m12 * (l*l + h*h);
                    I_yy = m12 * (w*w + h*h);
                    I_zz = m12 * (l*l + w*w);

                    % inertia tensor at the body's center of mass (CoM):
                    I_cm = inertiaTensorCM(I_xx, I_yy, I_zz);
                case 3
                    % cube:
                    s = varargin{1,1}; % side of the cube (in xyz-direction)

                    s2   = s*s;
                    m_rb = rho*(s2*s); % mass of the cube

                    % inertia tensor at CoM:
                    I_cm = (m_rb/6*s2) * eye(3,3);
                otherwise
                    error('massInertiaGObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'scyl'
            % solid cylinder:
            if (nargin ~= 4)
                error('massInertiaGObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            r = varargin{1,1}; % radius
            h = varargin{1,2}; % height

            r2   = r*r;
            V    = pi*r2*h; % volume of the solid cylinder
            m_rb = rho*V;   % mass of the solid cylinder

            I_xx = m_rb/12*(3*r2 + h*h); % I_yy = I_xx
            I_zz = 0.5*m_rb*r2;

            % inertia tensor at CoM:
            I_cm = inertiaTensorCM(I_xx, I_xx, I_zz);
        case 'cylt'
            % cylindrical tube:
            if (nargin ~= 5)
                error('massInertiaGObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            ri = varargin{1,1}; % inner radius
            ro = varargin{1,2}; % outer radius
            h  = varargin{1,3}; % height

            checkThickness(ri, ro);

            ro2 = ro*ro;
            ri2 = ri*ri;

            V    = pi*h*(ro2 - ri2); % volume of the cylindrical tube
            m_rb = rho*V; % mass of the cylindrical tube

            I_xx = m_rb/12*(3*(ri2 + ro2) + h*h); % I_yy = I_xx
            I_zz = 0.5*m_rb*(ri2 + ro2);

            % inertia tensor at CoM:
            I_cm = inertiaTensorCM(I_xx, I_xx, I_zz);
        case 'ssph'
            % solid sphere (ball):
            if (nargin ~= 3)
                error('massInertiaGObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            r = varargin{1,1}; % radius

            r2   = r*r;
            V    = 4/3*pi*(r2*r); % volume of the solid sphere
            m_rb = rho*V; % mass of the solid sphere

            % inertia tensor at CoM:
            I_cm = (2/5*m_rb*r2) * eye(3,3);
        case 'sphs'
            % spherical shell:
            if (nargin ~= 4)
                error('massInertiaGObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            ri = varargin{1,1};
            ro = varargin{1,2};

            checkThickness(ri, ro);

            % powers of ro & ri ...
            ro2 = ro*ro;
            ro3 = ro2*ro;
            ro5 = ro3*ro*ro;
            ri3 = ri*ri*ri;
            ri5 = ri3*ri*ri;

            V    = 4/3*pi*(ro3 - ri3); % volume of the spherical shell
            m_rb = rho*V; % mass of the spherical shell

            % inertia tensor at CoM:
            I_cm = (2/5*m_rb*( (ro5 - ri5) / (ro3 - ri3) )) * eye(3,3);
        otherwise
            error('massInertiaGObj: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
end
%% END of massInertiaGObj.


%% INERTIA TENSOR & CHECK FUNCTION:

function I_cm = inertiaTensorCM(I_xx, I_yy, I_zz)
    % create the inertia tensor of an object with the origin of
    % the coordinate system at the body's center of mass (CoM):
    I_cm = zeros(3,3);
    I_cm(1,1) = I_xx;
    I_cm(2,2) = I_yy;
    I_cm(3,3) = I_zz;
end

function checkThickness(ri, ro)
    % verify the thickness of the material ...
    if ((ro - ri) <= 0)
        error('massInertiaGObj: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
    end
end
