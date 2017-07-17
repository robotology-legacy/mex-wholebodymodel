function Mj = cartmass2jspc(a_J_c, Mx)
    %% Back-transformation of the Cartesian mass matrix from the
    %  operational space to the joint space:
    %
    % Note: The complete joint space matrix cannot be obtained. I.e. Mj(q) != M(q), only the
    % part which has an effect e.g. on the tool center point (TCP), the null-space component
    % is filtered out.
    aJc_t = a_J_c.';
    Mj = aJc_t * Mx * a_J_c; % mass matrix in joint coordinates.
end
