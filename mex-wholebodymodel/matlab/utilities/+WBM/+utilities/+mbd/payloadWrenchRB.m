function w_pl = payloadWrenchRB(M_pl, v_pl, a_pl, w_e) % RB ... Rigid Body
    % spatial cross operator for the mixed payload velocity in R^6 ...
    SCPv = WBM.utilities.tfms.mixvelcp(v_pl);

    % apply the Newton-Euler equation to calculate the payload
    % wrench in contact frame {C} with an external wrench w_e:
    % Note: All values must be from the same ref. frame.
    w_pl = (M_pl*a_pl) + (SCPv*M_pl*v_pl) + w_e;
end
