function f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk)
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

        % compute the (negated) total contact wrenches of
        % each hand in contact frame {c_i} = {lnk_i}:
        lnk_p_cm_l = obj.mwbm_config.payload_links(1,1).lnk_p_cm;
        lnk_p_cm_r = obj.mwbm_config.payload_links(1,2).lnk_p_cm;

        wc_tot_l = fhTotCWrench(fcp_l, lnk_R_cm, lnk_p_cm_l);
        wc_tot_r = fhTotCWrench(fcp_r, lnk_R_cm, lnk_p_cm_r);

        % payload forces f_pl of both hands (in contact frames {lnk_1} & {lnk_2})
        % with applied wrenches/forces of the robot to the object at each
        % contact point pc_i:
        % (the calculation is not really correct, but a good approximation)
        fp_l = dynPayloadForce(obj, 1, wf_v_lnk_l, wf_a_lnk_l) * 0.5;
        fp_r = dynPayloadForce(obj, 2, wf_v_lnk_r, wf_a_lnk_r) * 0.5;
        f_pl = vertcat(fp_l + wc_tot_l, fp_r + wc_tot_r);
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
    % normalize the payload forces ...
    n = sqrt(f_pl.'*f_pl);
    f_pl = f_pl / n;
end
