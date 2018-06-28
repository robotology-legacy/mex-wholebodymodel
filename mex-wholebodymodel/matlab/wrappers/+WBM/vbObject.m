% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

                                                                   % & WBM.wbmGObj (class incompatible)
classdef (Abstract) vbObject < handle & matlab.mixin.Heterogeneous
    % :class:`!vbObject` is an *abstract class* to specify *geometric volume body
    % objects* for the environment scenario of the robot simulation.
    %
    % Attributes:
    %   origin (double, vector): :math:`(3 \times 1)` Cartesian position of the
    %                            origin of the object. Default origin: :math:`[0, 0, 0]^T`.
    %   rotm   (double, matrix): :math:`(3 \times 3)` rotation matrix of the object.
    %                            If undefined, the default orientation is the
    %                            *identity matrix*.
    %   tform  (double, matrix): :math:`(4 \times 4)` transformation matrix of the
    %                            object. If :attr:`origin` and :attr:`rotm` are
    %                            undefined, then by default, the transformation
    %                            matrix is an *identity matrix*.
    %   frame  (double, vector): :math:`(7 \times 1)` VQ-transformation vector
    %                            (position and orientation) of the object. If
    %                            :attr:`origin` and :attr:`rotm` are undefined,
    %                            then the frame vector uses the default
    %                            transformation vector :attr:`DF_FRAME`.
    %
    %   DF_FRAME (double, vector): Default frame vector (VQ-transformation) of
    %                              the form :math:`[0, 0, 0, 1, 0, 0, 0]^T`
    %                              (*constant*).
    %
    %   description       (char, vector): Short description string about the
    %                                     volume body object (default: *empty*).
    %   ismovable      (logical, scalar): Boolean flag to indicate if the volume
    %                                     body object is movable (default: *false*).
    %   line_width      (double, scalar): Line width of the edges of the (geometric)
    %                                     volume body, specified as a positive value
    %                                     in points.
    %   edge_color (double/char, vector): Edge color of the (geometric) volume body,
    %                                     specified by a RGB-triplet or a color name.
    %   face_color (double/char, vector): Face color of the volume body, specified
    %                                     by a RGB-triplet or a color name.
    %   face_alpha      (double, scalar): Face transparency of the volume body,
    %                                     specified by a scalar in range :math:`[0,1]`.
    %
    %   obj_type      (char, vector): Type of the object (*read only*), specified by
    %                                 one of these values:
    %
    %                                    - ``'obs'``: The object defines an *obstacle* (for the robot).
    %                                    - ``'bdy'``: The object is only an arbitrary *volume body* in
    %                                      the environment and not a specific obstacle.
    %
    %                                 The default object type is ``'bdy'``.
    %   isobstacle (logical, scalar): Boolean flag to indicate if the volume body object
    %                                 is also defined as an obstacle (*read only*).
    %
    %                                 **Note:** The value will be set to *true*, if
    %                                 and only if :attr:`obj_type` is defined as an
    %                                 obstacle (*'obs'*), else *false* (default).
    %   issolid    (logical, scalar): Boolean flag to indicate if the volume body is
    %                                 a *solid* object (*read only*). Default: *false*.
    %
    %                                 **Note:** If a volumetric mass density is given,
    %                                 then the object is automatically defined as a
    %                                 *solid volume body*, i.e. the variable :attr:`!issolid`
    %                                 will be set to *true*.
    %   istool     (logical, scalar): Boolean flag to indicate if the volume body defines
    %                                 also a tool to be used by the robot (*read only*).
    %                                 Default: *false*.
    %   init_frame  (double, vector): :math:`(7 \times 1)` initial frame vector
    %                                 (VQ-transformation) of the object, in
    %                                 dependency of the given origin position
    %                                 and orientation (*read only*).
    %
    %                                 **Note:** If the attributes :attr:`origin` and
    %                                 :attr:`rotm` are not defined, then the initial
    %                                 frame uses the default transformation vector
    %                                 :attr:`DF_FRAME`.
    %   dimension   (double, vector): Row-vector to define the dimensions of the
    %                                 geometric volume body (*read only*).
    %
    %                                 **Note:** The form of the dimension vector
    %                                 depends strongly on the given shape of the
    %                                 volume body and can be different to each
    %                                 object.
    %   com   (double, vector): :math:`(3 \times 1)` Cartesian position of the
    %                           center of mass (CoM) of the volume body object
    %                           relative to the given origin (*read only*).
    %
    %                           **Note:** By default, the CoM of the object is
    %                           placed at the origin.
    %   rho   (double, scalar): Volumetric mass density of the object, specified as a
    %                           positive value in :math:`[\si{\kg/\m^3}]` (*read only*).
    %
    %                           **Note:** The object is defined as a *solid volume body*
    %                           if and only if the density value :math:`\rho > 0`. If the
    %                           density of the object is undefined, then the default
    %                           value is 0.
    %   m_rb  (double, scalar): Mass of the solid volume body object (rigid body
    %                           *rb*) in :math:`[\si{\kg}]` (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, the by
    %                           default the mass value is 0.
    %   I_cm  (double, matrix): :math:`(3 \times 3)` inertia matrix of the solid
    %                           volume body object at the center of mass *cm*
    %                           (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then the
    %                           inertia of the object is by default the *identity
    %                           matrix*.
    %   mgrid         (struct): Data structure for the *grid point coordinates* of
    %                           the internal 3D-meshgrid of the object (*read only*).
    %
    %                           The 3D-meshgrid will be used in the robot simulation
    %                           to simulate the volume body as a solid obstacle.
    %
    %                           **Note:** The data structure consists of the fields
    %                           ``X``, ``Y`` and ``Z`` that are representing the x,
    %                           y and z-coordinates over a grid as 3D-arrays with
    %                           three inputs. The meshgrid for the object can be
    %                           created only if the object is defined as an
    %                           *obstacle*.
    % Methods:
    %   setInitFrame(obj, varargin): *abstract* -- Sets the object to the given
    %                                              initial position and orientation.
    %   getGObj(obj): *abstract* -- Creates and draws the volume body object and returns
    %                               a handle to the created graphics objects.
    %   updGObj(obj, hgo): *abstract* -- Updates the data parameters, i.e. the vertex
    %                                    coordinates, of the volume body object.
    %   drawMGrid(obj, pt_color): *abstract* -- Draws the meshgrid of the object and
    %                                           returns a graphics object handle to it.
    %   ptInObj(obj, pt_pos): *abstract* -- Determines if some specified points
    %                                       are below the surface (i.e. inside)
    %                                       of the volume body object.
    % See Also:
    %   :class:`~WBM.vbCuboid`, :class:`~WBM.vbCylinder` and :class:`~WBM.vbSphere`.
    properties(Abstract, Dependent)
        origin@double vector
        rotm@double   matrix
        tform@double  matrix
        frame@double  vector
    end

    properties(Constant)
        DF_FRAME@double    vector = [0; 0; 0; 1; 0; 0; 0];
    end

    properties(Abstract)
        description@char
        ismovable@logical  scalar
        line_width@double  scalar
        edge_color
        face_color
        face_alpha@double  scalar
    end

    properties(Abstract, SetAccess = private, GetAccess = public)
        obj_type@char
        isobstacle@logical scalar
        issolid@logical    scalar
        istool@logical     scalar
        init_frame@double  vector
        dimension@double   vector
        com@double         vector
        rho@double         scalar
        m_rb@double        scalar
        I_cm@double        matrix
        mgrid@struct
    end

    methods(Abstract)
        setInitFrame(obj, varargin)
        hgo    = getGObj(obj)      % get the graphics object handle
        hgo    = updGObj(obj, hgo) % update the graphics object handle
        hmg    = drawMGrid(obj, pt_color)
        result = ptInObj(obj, pt_pos)
    end

    methods(Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement()
            % Returns the default volume body object for heterogeneous
            % array operations.
            %
            % Returns:
            %   default_object: An instance of :class:`~WBM.vbCuboid` with
            %   default values to preallocate the memory.
            default_object = WBM.vbCuboid;
        end

    end
end
