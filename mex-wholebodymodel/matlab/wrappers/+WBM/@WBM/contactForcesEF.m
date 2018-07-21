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

function [f_c, tau_gen] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j)
    % Computes the *contact forces* :math:`f_c` of the given *contact constraints*
    % with *external forces* (*EF*) that are acting on the contact links of the
    % robot.
    %
    % The formula for calculating the *contact constraint forces* is derived
    % from the dynamic equations of motion of the robot, such that:
    %
    %   .. math::
    %      :label: contact_forces_ef
    %
    %      f_c = \Lambda_c (a_c + J_c M^{\text{-}1} (C(q_j, \dot{q}_j) -
    %      \tau_{gen}) - \dot{J}_c\dot{q}_j) - f^{\small C}_e
    %
    % The variable :math:`\Lambda_c = \Upsilon_{c}^{\text{-}1} = (J_c M^{\text{-}1} J_{c}^{T})^{\text{-}1}`
    % denotes the *inertia matrix* (or *pseudo-kinetic energy matrix*
    % :cite:`Khatib1987`) in contact space :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`,
    % :math:`\tau_{gen} = S_j (\tau - \tau_{fr})` represents the
    % *generalized forces* with the *joint selection matrix*
    % :math:`S_j = [\mathbf{0}_{(6 \times n)}\enspace \mathbb{I}_n]^T` and the
    % *friction forces* :math:`\tau_{fr}`. The variable :math:`a_c` denotes
    % the *mixed contact accelerations* and :math:`f^{\small C}_e` represents
    % the *external forces* that are acting on the *contact links* in contact
    % space :math:`\mathrm{C}`.
    %
    % The method can be called as follows:
    %
    %   .. py:method:: contactForcesEF(tau, fe_c, ac, Jc, djcdq, M, c_qv[, dq_j])
    %
    % Arguments:
    %   tau   (double, vector): :math:`(n \times 1)` torque force vector for the
    %                           joints and the base of the robot.
    %   fe_c  (double, vector): :math:`(k \times 1)` vector of external forces (in
    %                           contact space) that are acting on the specified
    %                           contact links with size of :math:`k = 6` (one link)
    %                           or :math:`k = 12` (both links).
    %   ac    (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                           for the *contact points* of the specified contact
    %                           links with the size of :math:`k = 6` (one link) or
    %                           :math:`k = 12` (both links).
    %
    %                           **Note:** If the given accelerations are very
    %                           small, then the vector can also be *constant*
    %                           or *zero*.
    %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
    %                           *contact constraints* in contact space
    %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
    %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
    %                           the time derivative of the contact Jacobian
    %                           :math:`\dot{J}_c` and the joint velocity
    %                           :math:`\dot{q}_j`.
    %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
    %                           of the robot.
    %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
    %                           vector of the robot.
    %   dq_j  (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                           vector in :math:`[\si{\radian/s}]` (*optional*).
    %
    %                           **Note:** If the joint velocity is not given,
    %                           then the method assumes that the robot system
    %                           is *frictionless*.
    %
    % The variable :math:`m` denotes the *number of contact constraints* of
    % the robot and :math:`n = n_{dof} + 6`.
    %
    % Returns:
    %   [f_c, tau_gen]: 2-element tuple containing:
    %
    %      - **f_c**     (*double, vector*) -- :math:`(n \times 1)` contact
    %        constraint force vector,
    %      - **tau_gen** (*double, vector*) -- :math:`(n \times 1)` generalized
    %        force vector of the robot,
    %
    %   with :math:`n = n_{dof} + 6`.
    %
    % See Also:
    %   :meth:`WBM.contactForces`, :meth:`contactForcesCLPCEF` and
    %   :meth:`WBMBase.frictionForces`.
    %
    % References:
    %   - :cite:`Park2006`, chapter 5, pp. 106-110, eq. (5.5)-(5.14).
    %   - :cite:`Murray1994`, pp. 269-270, eq. (6.5) and (6.6).
    %   - :cite:`Khatib1987`, p. 49-50, eq. (51).

    % References:
    %   [Par06] Park, Jaeheung: Control Strategies for Robots in Contact.
    %           PhD-Thesis, Artificial Intelligence Laboratory, Stanford University,
    %           2006, Chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %           URL: <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [MLS94] Murray, R. M.; Li, Z.; Sastry, S. S.: A Mathematical Introduction to Robotic Manipulation.
    %           CRC Press, 1994, pp. 269-270, eq. (6.5) and (6.6).
    %   [Kha87] Khatib, Oussama: A unified approach for motion and force control of robot manipulators: The operational space formulation.
    %           In: IEEE Journal on Robotics and Automation, Volume 3, Issue 1, 1987, p. 49-50, eq. (51),
    %           URL: <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1987_RA.pdf>.
    switch nargin
        % fe_c ... external forces that are acting on the contact links (in contact space)
        % ac   ... mixed contact accelerations
        case 9
            % generalized forces with friction:
            tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
            tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.
        case 8
            % general case:
            tau_gen = tau;
        otherwise
            error('WBM::contactForcesEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    if (size(tau,1) < size(c_qv,1))
        tau_gen = vertcat(zeros(6,1), tau_gen);
    end
    % Calculation of the contact (constraint) force vector:
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % = Jc * M^(-1)
    Upsilon_c = JcMinv * Jc_t; % inverse mass matrix Upsilon_c = Lambda^(-1) = Jc * M^(-1) * Jc^T in contact space {c},
                               % Lambda^(-1) ... inverse pseudo-kinetic energy matrix.
    % contact constraint forces f_c (generated by the environment):
    f_c = (Upsilon_c \ (ac + JcMinv*(c_qv - tau_gen) - djcdq)) - fe_c;
end
