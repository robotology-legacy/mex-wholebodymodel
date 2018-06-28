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

function dstvChi = forwardDynamicsNFBEF(obj, t, stvChi, fhTrqControl, clnk_conf, fe_c, ac)
    % Computes the *forward dynamics* of the *whole body model* of a humanoid
    % robot with *external forces* that are acting on the contact links of the
    % robot, but without a *floating base* (*NFBEF*) or by exclusion of it --
    % *experimental*.
    %
    % Since the given robot is defined without a floating base, there are no
    % contact forces :math:`f_c` to the ground, i.e. :math:`f_c = 0`. The
    % behavior of the model is the same as a robot with a fixed base.
    %
    % The forward dynamics describes the calculation of the acceleration response
    % :math:`\ddot{q}` of a rigid-body system to a given torque force :math:`\tau`
    % (:cite:`Featherstone2008`, p. 2). The dynamic model will computed with an
    % *ordinary differential equation* (ODE) as defined in
    % :eq:`forward_dynamics_chi_ef`.
    %
    % The method will be passed to a Matlab ODE solver :cite:`Shampine1997` or
    % other solvers for stiff differential equations and differential-algebraic
    % systems (differential equations with constraints).
    %
    % Note:
    %   This forward dynamics method is still untested and should be considered
    %   as *experimental*.
    %
    % Arguments:
    %   t             (double, scalar): Current time step of the given time interval
    %                                   of integration :math:`[t_0, t_f]` with
    %                                   :math:`n > 1` step elements in ascending order.
    %   stvChi        (double, vector): :math:`((2 n_{dof} + 13) \times 1)` state vector
    %                                   :math:`\chi` of the robot model at the current
    %                                   time step :math:`t` that will be integrated by
    %                                   ODE solver.
    %   fhTrqControl (function_handle): Function handle to a specified time-dependent
    %                                   *torque control function* that controls the
    %                                   dynamics of the robot system.
    %   clnk_conf             (struct): Configuration structure to specify the
    %                                   *qualitative state* of at most two
    %                                   *contact links*.
    %
    %                                   The data structure specifies which link
    %                                   is currently in contact with the ground
    %                                   or an object.
    %   fe_c          (double, vector): :math:`(k \times 1)` vector of external forces
    %                                   (in contact space :math:`\mathrm{C}`) that are
    %                                   acting on the specified contact links.
    %   ac            (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                                   for the *contact points* of the contact links.
    %
    % The variable :math:`k` indicates the *size* of the given *force* and *acceleration
    % vectors* in dependency of the specified contact links:
    %
    %   - :math:`k = 6`  -- only one link is defined.
    %   - :math:`k = 12` -- both links are defined.
    %
    % The given *external forces* and *accelerations* are either *constant* or *zero*.
    %
    % Returns:
    %   dstvChi (double, vector): :math:`((2 n_{dof} + 13) \times 1)` time-derivative vector
    %   (next state) of the state function :math:`\chi(t)` at the current time step :math:`t`
    %   satisfying the equation :eq:`forward_dynamics_chi_ef`.
    %
    % See Also:
    %   :meth:`WBM.forwardDynamicsNFB` and :meth:`WBM.forwardDynamicsNFBPL`.
    %
    % References:
    %   - :cite:`Featherstone2008`, p. 2 and p. 41, eq. (3.3).
    %   - :cite:`Shampine1997`

    % References:
    %   [Fea08] Featherstone, Roy: Rigid Body Dynamics Algorithms. Springer, 2008,
    %           p. 2 and p. 41, eq. (3.3).
    %   [LR97]  Shampine, L. F.; Reichelt, M. W.: The Matlab ODE Suite.
    %           In: SIAM Journal on Scientific Computing, Volume 18, Issue 1, 1997,
    %           URL: <https://www.mathworks.com/help/pdf_doc/otherdocs/ode_suite.pdf>.

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    wf_omega_b = stp.omega_b;
    v_b = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity

    % update the state for the optimized mode
    % (else the procedure computes nonsense) ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv] = wholeBodyDyn(obj); % optimized mode

    % get the current control torques ...
    tau = fhTrqControl(t, M, c_qv, stp);

    % new mixed generalized velocity vector ...
    nu  = fdynNewMixedVelocities(obj, stp.qt_b, stp.dx_b, wf_omega_b, stp.dq_j);
    % joint acceleration dnu = ddq_j (optimized mode):
    dnu = jointAccelerationsNFBEF(obj, clnk_conf, tau, fe_c, ac, M, c_qv, stp.dq_j);

    dstvChi = vertcat(nu, dnu);
end
