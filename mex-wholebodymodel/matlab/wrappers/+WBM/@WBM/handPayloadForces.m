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

function f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk, norm_pl)
    % Calculates the *payload forces* (wrenches) :math:`f_{pl}` of a humanoid
    % robot at the specified *payload links* of the hands in contact (operational)
    % space :math:`\mathrm{C}_h = \mathrm{C}_{lnk}`.
    %
    % By default, the calculated payload forces will be *normalized* between
    % :math:`[0, 1]`.
    %
    % Arguments:
    %   hand_conf             (struct): Configuration structure to specify the
    %                                   *qualitative state* of the hands.
    %
    %                                   The data structure specifies which hand
    %                                   is currently in contact with the payload
    %                                   object.
    %   fhTotCWrench (function_handle): Function handle to a specific *total
    %                                   contact wrench function* in *contact space*
    %                                   :math:`\mathrm{C_h = \{C_1,\ldots,C_n\}}`
    %                                   of the hands that will be applied by the
    %                                   robot model.
    %   f_cp          (double, vector): Force vector or scalar applied to a grasped
    %                                   object at the *contact points*
    %                                   :math:`{}^{\small O}p_{\small C_i}` from the
    %                                   contact frames :math:`\mathrm{\{C_1,\ldots,C_n\}}`
    %                                   of the hands to the origin frame :math:`\mathrm{O}`
    %                                   at the CoM of the object.
    %
    %                                   The vector length :math:`l` of the applied
    %                                   forces depends on the chosen *contact model*
    %                                   and if only one hand or both hands are involved
    %                                   in grasping an object, such that :math:`l = h\cdot s`
    %                                   with size :math:`s \in \{1,3,4\}` and the number
    %                                   of hands :math:`h \in \{1,2\}`.
    %
    %                                   **Note:** The z-axis of a contact frame
    %                                   :math:`\mathrm{C_{i \in \{1,\ldots,n\}}}`
    %                                   points in the direction of the inward surface
    %                                   normal at the point of contact
    %                                   :math:`{}^{\small O}p_{\small C_i}`. If the
    %                                   chosen contact model is *frictionless*, then
    %                                   each applied force to the object is a scalar,
    %                                   otherwise a vector.
    %   wf_v_lnk      (double, vector): :math:`(k \times 1)` mixed velocity vector
    %                                   from the link frame *lnk* to world frame *wf*.
    %   wf_a_lnk      (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                                   from the link frame *lnk* to the world frame *wf*.
    %   norm_pl      (logical, scalar): Boolean flag to indicate if the calculated payload
    %                                   forces will be normalized (default: *true*) --
    %                                   *optional*.
    %
    %                                   If the flag is set to *false*, then the payload
    %                                   values will not be normalized.
    %
    % The variable :math:`k` indicates the *size* of the given *acceleration vectors*
    % in dependency of the specified hands:
    %
    %   - :math:`k = 6`  -- only one hand is defined.
    %   - :math:`k = 12` -- both hands are defined/undefined.
    %
    % The order of the specified payload links (hands) is from left to right.
    %
    % Returns:
    %   f_pl (double, vector): :math:`(k \times 1)` payload force vector (normalized or
    %   unnormalized) with the size of :math:`k = 6` or :math:`k = 12`.
    %
    % See Also:
    %   :class:`~WBM.wbmPayloadLink` and :meth:`WBM.dynPayloadForce`.
    switch nargin
        case 6
            % normalize the payload forces (default)
            norm_pl = true;
        case 7
            if ~islogical(norm_pl)
                error('WBM::handPayloadForces: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        otherwise
            error('WBM::handPayloadForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    ctc_l = hand_conf.contact.left;
    ctc_r = hand_conf.contact.right;
    lnk_R_cm = eye(3,3);

    % check which link (hand) has contact with the object(s):
    if (ctc_l && ctc_r)
        % both hands:
        wf_a_lnk_l = wf_a_lnk(1:6,1);  % accelerations
        wf_a_lnk_r = wf_a_lnk(7:12,1);
        wf_v_lnk_l = wf_v_lnk(1:6,1);  % velocities
        wf_v_lnk_r = wf_v_lnk(7:12,1);

        n = length(f_cp);
        if (mod(n,2) ~= 0), error('handPayloadForces: The vector length is not even!'); end
        sp = n*0.5; % split position
        fcp_l = f_cp(1:sp);
        fcp_r = f_cp((sp+1):n);

        % compute the (external) total contact wrenches of
        % each hand in contact frame {c_i} = {lnk_i}:
        lnk_p_cm_l = obj.mwbm_config.payload_links(1,1).lnk_p_cm;
        lnk_p_cm_r = obj.mwbm_config.payload_links(1,2).lnk_p_cm;

        wc_tot_l = fhTotCWrench(fcp_l, lnk_R_cm, lnk_p_cm_l);
        wc_tot_r = fhTotCWrench(fcp_r, lnk_R_cm, lnk_p_cm_r);

        % payload forces f_pl of both hands (in contact frames {lnk_1} & {lnk_2})
        % minus the applied (external) wrenches/forces of the robot to the object
        % at each contact point pc_i:
        % note: the calculation is not really correct, but a good approximation.
        fp_l = dynPayloadForce(obj, 1, wf_v_lnk_l, wf_a_lnk_l) * 0.5;
        fp_r = dynPayloadForce(obj, 2, wf_v_lnk_r, wf_a_lnk_r) * 0.5;
        f_pl = vertcat(fp_l - wc_tot_l, fp_r - wc_tot_r);
    elseif ctc_l
        % only left hand:
        lnk_p_cm = obj.mwbm_config.payload_links(1,1).lnk_p_cm;

        % total contact wrench of the left hand (at {c_1} = {lnk_1}):
        wc_tot_l = fhTotCWrench(f_cp, lnk_R_cm, lnk_p_cm);
        % get the payload force of the left hand:
        f_pl = dynPayloadForce(obj, 1, wf_v_lnk, wf_a_lnk, wc_tot_l);
    elseif ctc_r
        % only right hand:
        lnk_p_cm_r = obj.mwbm_config.payload_links(1,2).lnk_p_cm;

        % total contact wrench of the right hand (at {c_2} = {lnk_2}):
        wc_tot_r = fhTotCWrench(f_cp, lnk_R_cm, lnk_p_cm_r);
        % get the payload force of the right hand:
        f_pl = dynPayloadForce(obj, 2, wf_v_lnk, wf_a_lnk, wc_tot_r);
    else
        % no contact:
        f_pl = obj.ZERO_CVEC_12;
        return
    end

    if norm_pl
        % normalize between [0, 1] ...
        n = sqrt(f_pl.'*f_pl);
        f_pl = f_pl / n;
    end
end
