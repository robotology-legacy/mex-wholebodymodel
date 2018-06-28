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

classdef wbmRobotModel < handle
    % :class:`!wbmRobotModel` is a *data type* (class) that specifies the model
    % parameters of a given floating-base robot.
    %
    % Attributes:
    %   yarp_robot_type   (char, vector): String to specify the type of a YARP-based
    %                                     robot (default type: ``'iCub'``).
    %   urdf_robot_name   (char, vector): String matching *name* of an existing robot model
    %                                     [#f5]_, or a string with the full path to a
    %                                     specific URDF-file [#f6]_ to be read.
    %   ndof            (uint16, scalar): Number of degrees of freedom of the given robot.
    %   urdf_fixed_link   (char, vector): String matching URDF name of the *fixed reference
    %                                     link frame* w.r.t. a world frame (wf). The given
    %                                     reference link of the robot will be used as the
    %                                     *floating-base link*.
    %
    %                                     **Note:** The default fixed link (floating-base link)
    %                                     of each YARP-based robot can be different and the
    %                                     selection of the floating-base link depends also from
    %                                     the situation. For example the default fixed link of
    %                                     the iCub humanoid robot is ``l_sole``.
    %   wf_R_b_init     (double, matrix): :math:`(3 \times 3)` rotation matrix as *initial orientation*
    %                                     from the base frame *b* to the world frame *wf*. The default
    %                                     orientation is the *identity matrix*.
    %   wf_p_b_init     (double, vector): :math:`(3 \times 1)` vector to specify the *initial position*
    %                                     from the base frame *b* to the world frame *wf*. The default
    %                                     position is :math:`[0, 0, 0]^T`.
    %   g_wf            (double, vector): :math:`(3 \times 1)` Cartesian gravity vector in the world
    %                                     frame *wf*. If the gravity is not defined, then the gravity
    %                                     vector is by default a 0-vector.
    %   frict_coeff             (struct): Data structure for the friction coefficients of
    %                                     of the joints of the given robot model, specified
    %                                     by the field variables:
    %
    %                                        - ``v``: :math:`(n_{dof} \times 1)` coefficient vector for the *viscous friction*.
    %                                        - ``c``: :math:`(n_{dof} \times 1)` coefficient vector for the *Coulomb friction*.
    %
    %   jlmts                   (struct): Data structure for the *joint positions limits* of the
    %                                     given robot model, specified by the field variables:
    %
    %                                        - ``lwr``: :math:`(n_{dof} \times 1)` vector for the *lower* joint limits.
    %                                        - ``upr``: :math:`(n_{dof} \times 1)` vector for the *upper* joint limits.
    % See Also:
    %   :class:`~WBM.WBMBase` and :class:`~WBM.WBM`.
    properties
        yarp_robot_type@char      = 'iCub';
        urdf_robot_name@char
        ndof@uint16        scalar = 0;
        urdf_fixed_link@char
        wf_R_b_init@double matrix = eye(3,3);
        wf_p_b_init@double vector = zeros(3,1);
        g_wf@double        vector = zeros(3,1);
        frict_coeff = struct( 'v', [], ...
                              'c', [] );
        jlmts       = struct( 'lwr', [], ...
                              'upr', [] );
    end
end
