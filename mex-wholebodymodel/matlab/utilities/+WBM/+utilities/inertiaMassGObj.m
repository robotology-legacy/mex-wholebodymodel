function [I_cm, m_rb] = inertiaMassGObj(obj_type, rho, varargin)
    %% Returns the inertia tensor and the corresponding mass of a geometrical object with
    %  the origin of the coordinate system at the object body's center of mass (CoM):
    %
    %  Sources:
    %   [1]  A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 163.
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
                    error('inertiaMassGObj: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        case 'scyl'
            % solid cylinder:
            if (nargin ~= 4)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            r = varargin{1,1}; % radius
            h = varargin{1,2}; % height

            r2   = r*r;
            V    = pi*r2*h; % volume of the solid cylinder
            m_rb = rho*V; % mass of the solid cylinder

            I_xx = m_rb/12*(3*r2 + h*h); % I_yy = I_xx
            I_zz = 0.5*m_rb*r2;

            % inertia tensor at CoM:
            I_cm = inertiaTensorCM(I_xx, I_xx, I_zz);
        case 'cylt'
            % cylindrical tube:
            if (nargin ~= 5)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            r_o = varargin{1,1}; % outer radius
            h   = varargin{1,2}; % height
            t_m = varargin{1,3}; % thickness of the material

            if (t_m <= 0)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end
            r_i = r_o - t_m; % inner radius

            ro2 = r_o*r_o;
            ri2 = r_i*r_i;

            V    = pi*h*(ro2 - ri2); % volume of the cylindrical tube
            m_rb = rho*V; % mass of the cylindrical tube

            I_xx = m_rb/12*(3*(ri2 + ro2) + h*h); % I_yy = I_xx
            I_zz = 0.5*m_rb*(ri2 + ro2);

            % inertia tensor at CoM:
            I_cm = inertiaTensorCM(I_xx, I_xx, I_zz);
        case 'ssph'
            % solid sphere:
            if (nargin ~= 3)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            r = varargin{1,1}; % radius

            r2   = r*r;
            V    = 4/3*pi*(r2*r); % volume of the solid sphere
            m_rb = rho*V; % mass of the solid sphere

            % inertia tensor at CoM:
            I_cm = (2/5*m_rb*r2) * eye(3,3);
        case 'sphs'
            % sphere shell:
            if (nargin ~= 4)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            r_o = varargin{1,1}; % outer radius
            t_m = varargin{1,2}; % thickness of the material

            if (t_m <= 0)
                error('inertiaMassGObj: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end
            r_i = r_o - t_m; % inner radius

            % powers of r_o & r_i ...
            ro2 = r_o*r_o;
            ro3 = ro2*r_o;
            ro5 = ro3*r_o*r_o;
            ri3 = r_i*r_i*r_i;
            ri5 = ri3*r_i*r_i;

            V    = 4/3*pi*(ro3 - ri3); % volume of the sphere shell
            m_rb = rho*V; % mass of the sphere shell

            % inertia tensor at CoM:
            I_cm = (2/5*m_rb*( (ro5 - ri5) / (ro3 - ri3) )) * eye(3,3);
        otherwise
            error('inertiaMassGObj: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
end
%% END of inertiaMassGObj.


%% INERTIA TENSOR:

function I_cm = inertiaTensorCM(I_xx, I_yy, I_zz)
    % create the inertia tensor of an object with the origin of
    % the coordinate system at the body's center of mass (CoM):
    I_cm = zeros(3,3);
    I_cm(1,1) = I_xx;
    I_cm(2,2) = I_yy;
    I_cm(3,3) = I_zz;
end
