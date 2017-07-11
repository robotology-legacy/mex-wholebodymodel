function [f_pl, pl_prms] = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, v_pl, a_pl)
    ctc_l = hand_conf.contact.left;
    ctc_r = hand_conf.contact.right;

    % check which link (hand) has contact with the object(s):
    if (ctc_l && ctc_r)
        % both hands:
        % accelerations ...
        ap_l = a_pl(1:6,1);
        ap_r = a_pl(7:12,1);
        % velocities ...
        vp_l = v_pl(1:6,1);
        vp_r = v_pl(7:12,1);

        % calculate the generalized inertia for each hand:
        wf_H_ee = cell(1,2);
        M_p     = wf_H_ee;
        for i = 1:2
            [M_p{1,i}, frms] = generalizedInertiaPL(obj, i); % optimized mode
            wf_H_ee{1,i} = frms.wf_H_lnk;
        end

        % calculate the total contact wrench for each hand:
        n  = size(f_cp,1);
        sp = n*0.5; % split position
        fcp_l = f_cp(1:sp,1);
        fcp_r = f_cp((sp+1):n,1);

        % get the positions & orientations from the contact frame {c_i}
        % (at contact point pc_i) to the world frame {wf}:
        [wf_R_cl, wf_p_cl] = WBM.utilities.tform2posRotm(wf_H_ee{1,1});
        [wf_R_cr, wf_p_cr] = WBM.utilities.tform2posRotm(wf_H_ee{1,2});

        wc_tot_l = fhTotCWrench(fcp_l, wf_R_cl, wf_p_cl);
        wc_tot_r = fhTotCWrench(fcp_r, wf_R_cr, wf_p_cr);

        % payload forces of both hands:
        fp_l = payloadForce(obj, M_p{1,1}, vp_l, ap_l, wc_tot_l);
        fp_r = payloadForce(obj, M_p{1,2}, vp_r, ap_r, wc_tot_r);
        f_pl = vertcat(fp_l, fp_r);

        if (nargout == 2)
            wc_tot = vertcat(wc_tot_l, wc_tot_r);
            M_pl   = vertcat(M_p{1,1}, M_p{1,2});
            pl_prms = struct('wc_tot', wc_tot, 'M_pl', M_pl);
        end
    elseif ctc_l
        % only left hand:
        % compute the generalized inertia for the left hand:
        [M_pl, frms] = generalizedInertiaPL(obj, 1); % optimized mode
        wf_H_eel = frms.wf_H_lnk;

        % positions & orientations of the left contact point ...
        [wf_R_cl, wf_p_cl] = WBM.utilities.tform2posRotm(wf_H_eel);
        % calculate the total contact wrench of the left hand ...
        wc_tot_l = fhTotCWrench(f_cp, wf_R_cl, wf_p_cl);
        % get the payload force of the left hand:
        f_pl = payloadForce(obj, M_pl, v_pl, a_pl, wc_tot_l);

        if (nargout == 2)
            pl_prms = struct('wc_tot', wc_tot_l, 'M_pl', M_pl);
        end
    elseif ctc_r
        % only right hand:
        % compute the generalized inertia for the right hand:
        [M_pl, frms] = generalizedInertiaPL(obj, 2); % optimized mode
        wf_H_eer = frms.wf_H_lnk;

        % positions & orientations of the right contact point ...
        [wf_R_cr, wf_p_cr] = WBM.utilities.tform2posRotm(wf_H_eer);
        % calculate the total contact wrench of the right hand ...
        wc_tot_r = fhTotCWrench(f_cp, wf_R_cr, wf_p_cr);
        % get the payload force of the right hand:
        f_pl = payloadForce(obj, M_pl, v_pl, a_pl, wc_tot_r);

        if (nargout == 2)
            pl_prms = struct('wc_tot', wc_tot_r, 'M_pl', M_pl);
        end
    else
        % no contact:
        f_pl = obj.ZERO_CVEC_12;
        if (nargout == 2)
            pl_prms = struct('wc_tot', obj.ZERO_CVEC_12, 'M_pl', zeros(12,6));
        end
    end
end
