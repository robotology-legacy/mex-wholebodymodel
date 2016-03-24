function visualizeForwardDynamics(obj, x_out, tspan, sim_config, vis_speed)
    if ~exist('vis_speed', 'var')
        % use the default visualization speed value ...
        % (note: use another speed value in order to make the visualization speed
        %        close to real-time when the simulation time step is changed.)
        vis_speed = 1.0; 
    end

    % check the dimension and get the number of instances of the simulation result ...
    ndof = obj.mwbm_config.ndof;
    vlen = ndof + 7;
    [nRes, len] = size(x_out); 
    if (len ~= vlen)
        error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % get the translation, orientation and joint positions from the output vector
    % "x_out" of the integration part of the forward dynamics function:
    vqT_b = x_out(1:nRes,1:7);
    q_j = x_out(1:nRes,8:vlen);
                      
    v_b = vertcat(obj.mwbm_config.initStateParams.dx_b, obj.mwbm_config.initStateParams.omega_b);

    % obj.setWorldFrame2FixedLink(obj.mwbm_config.cstrLinkNames{1}, obj.mwbm_config.initStateParams.q_j, ...
    %                             obj.mwbm_config.initStateParams.dq_j, v_b, obj.mwbm_params.g_wf);

    obj.setWorldFrame2FixedLink(obj.mwbm_config.initStateParams.q_j, obj.mwbm_config.initStateParams.dq_j, ...
                                v_b, obj.mwbm_params.g_wf);

    nJnts = sim_config.robot_body.nJoints; % number of nodes (virtual joints) to be plotted
    nLnks = sim_config.robot_body.nLinks;  % number of edges (virtual links) to be plotted
    nFeets = sim_config.robot_body.nFeets; % number of feets of the robot's body

    %% Forward Kinematics of the robot's skeleton:

    % create a data structure for the forward kinematics + memory allocation:
    %
    % forward kinematic translation (xyz-position):
    fwd_kin.vtr = zeros(nJnts,3);
    % 3D data-points of the forward kin. translations (positions):
    fwd_kin.hTr_dp = zeros(1,nJnts);
    % joint pair (link) translations (xyz-positions):
    fwd_kin.jnt_pair_pos = zeros(nLnks,6);
    % forward kin. roto-translation (in VQS-form):
    fwd_kin.vqT = zeros(nRes,7,nJnts);

    % calculate the forward kinematic roto-translation
    % of each joint of the robot's joint name list:

    %fwd_kin.vqT(1:nRes,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ...

    for i = 1:nRes % for each result ...
        for j = 1:nJnts
            % convert the state of the base in roto-translation form ...
           [p_b, R_b] = WBM.utilities.frame2posRotm( squeeze(vqT_b(i,1:7)') );
           % set the world frame to the base ...
           obj.setWorldFrame(R_b, p_b, obj.mwbm_params.g_wf);
           % forward kinematic each joint (link) in the list ...
           fwd_kin.vqT(i,1:7,j) = (obj.forwardKinematics(sim_config.robot_body.joint_names{j}, R_b, p_b, q_j(i,1:ndof)'))';
        end
    end
    fwd_kin.vqT(1:nRes,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ...

    %traj_data = zeros(size(tspan,1),3); % init the trajectory vectors of the given task ...

    %% Initial plots of the robot skeleton:

    % clear and reset the subplots of the main figure in the reverse order:
    new_patch_color = [201 220 222] ./ 255;
    for i = 4:-1:1
        set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(i)); % set the current axes
        cla; % delete all graphics objects from the current axes
        axis off;
        % redraw the rectangle with a new color ...
        patch(sim_config.PATCH_SHAPE(1,1:4), sim_config.PATCH_SHAPE(2,1:4), ...
              sim_config.PATCH_SHAPE(3,1:4), new_patch_color);
        drawnow; % update and draw the current subplot in the main figure
    end
    % set back the current axes handle of the main figure to first subplot ...
    %set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(1));

    %%%%%%
    % plot the initial base position of the robot skeleton:
    mkr = '.';
    mkr_color = 'b'; % 'w';
    %fwd_kin.vtr(1,1:3) = fwd_kin.vqT(1,1:3,1);
    %fwd_kin.hTr_dp(1,1) = plot3(fwd_kin.vtr(1,1), fwd_kin.vtr(1,2), fwd_kin.vtr(1,3), 'Marker', mkr, 'MarkerEdgeColor', mkr_color);

    % plot the joints of the robot skeleton:
    %for i = 2:nJnts-1
    for i = 1:nJnts-1
        [fwd_kin.vtr(i,1:3), fwd_kin.hTr_dp(1,i)] = plotFKJointPos((fwd_kin.vqT(1,1:7,i))', mkr, mkr_color);
    end
    % plot the position of the center of mass (COM):
    [fwd_kin.vtr(nJnts,1:3), fwd_kin.hTr_dp(1,nJnts)] = plotFKJointPos((fwd_kin.vqT(1,1:7,nJnts))', '*', 'r');

    % create a position parameter matrix from the defined joint pairs (links) to describe a full
    % configuration of the robot's skeleton:
    fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin, sim_config.robot_body.joint_pair_idx{1,1}, nLnks);

    % (**)
    % (***)

    [body.hShape, body.hLnk_lns] = createRobotBody(nLnks, fwd_kin.jnt_pair_pos, nFeets, ...
                                                   sim_config.robot_body.foot_joints, sim_config.robot_body.foot_ds, ...
                                                   sim_config.robot_body.hull_size_sf, sim_config.robot_body.hull_faces);

    % delete the plot objects in the subplot nr. 2, 3 and 4 of the main figure ...
    if ~isempty(sim_config.plot_objs)    
        delete(sim_config.plot_objs{2});
        delete(sim_config.plot_objs{3});
        delete(sim_config.plot_objs{4});
    end

    % store all axes handle objects into a vector ...
    sim_config.plot_objs{1} = vertcat((body.hShape)', (body.hLnk_lns)', (fwd_kin.hTr_dp)');
    %sim_config.plot_objs{1} = [(body.hShape)'; (body.hLnk_lns)'; (fwd_kin.hTr_dp)'];

    % copy all objects to the other axes with different views:
    % view top?
    set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(2));
    hRootObj = get(sim_config.hAxes(2), 'Parent');
    sim_config.plot_objs{2} = copyobj(sim_config.plot_objs{1}, hRootObj);
    view(-90,90); % change the viewpoint (azimut, elevation)
    % view 
    set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(3));
    hRootObj = get(sim_config.hAxes(3), 'Parent');
    sim_config.plot_objs{3} = copyobj(sim_config.plot_objs{1}, hRootObj);
    view(0,1);
    % view 
    set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(4));
    hRootObj = get(sim_config.hAxes(2), 'Parent');
    sim_config.plot_objs{4} = copyobj(sim_config.plot_objs{1}, hRootObj);
    %sim_config.plot_objs{4} = copyobj(sim_config.plot_objs{1}, sim_config.hAxes(4));
    view(-90,1);
    % set back the current axes handle to the first subplot ...
    set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(1));

    %drawnow; % draw all plots of the initial robot body in the main figure ... 

    %% Update the plots of the robot's body:
    %

    t = 2;
    while (t <= nRes) % the visualization instance ...
        tic; % visualizer step timer start (to setting the visualizer speed) ???

        % update the forward kinematic translations (positions) ...
        new_fk_pos = squeeze(fwd_kin.vqT(t,1:7,1:nJnts));
        %fwd_kin.hTr_dp = updateFKJointPositions(fwd_kin.hTr_dp, new_fk_pos', nJnts);
        fwd_kin.hTr_dp = updateFKJointPositions(fwd_kin.hTr_dp, new_fk_pos, nJnts);

        % update the position parameter matrix for the links of the robot with the 2nd joint pair list ...
        fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin, sim_config.robot_body.joint_pair_idx{2,1}, nLnks);

        % update the edges of the skeleton with the respect to the new joint positions ...
        [body.hShape, body.hLnk_lns] = updateRobotBody(body.hShape, body.hLnk_lns, nLnks, fwd_kin.jnt_pair_pos, nFeets, ...
                                                       sim_config.robot_body.foot_joints, sim_config.robot_body.foot_ds, ...
                                                       sim_config.robot_body.hull_size_sf);

        % store the plot vector with the updated axes handle objects ...
        sim_config.plot_objs{1} = vertcat((body.hShape)', (body.hLnk_lns)', (fwd_kin.hTr_dp)');

        % delete the previous plot objects and copy the new updated plot objects to
        % the other axes for different views (subplots):
        % view ???
        delete(sim_config.plot_objs{2});
        hRootObj = get(sim_config.hAxes(2), 'Parent');
        sim_config.plot_objs{2} = copyobj(sim_config.plot_objs{1}, hRootObj);
        % view ???
        delete(sim_config.plot_objs{3});
        hRootObj = get(sim_config.hAxes(3), 'Parent');
        sim_config.plot_objs{3} = copyobj(sim_config.plot_objs{1}, hRootObj);
        % view ???
        delete(sim_config.plot_objs{4});
        hRootObj = get(sim_config.hAxes(4), 'Parent');
        sim_config.plot_objs{4} = copyobj(sim_config.plot_objs{1}, hRootObj);

        drawnow;

        % update the visualization speed to keep the simulation very close to the real time ...
        tdiff = vis_speed * sim_config.sim_step - toc();
        if (tdiff > 0)
            % make short break ...
            pause(tdiff);
        else
            % increase the visualization speed by 1 ...
            vis_speed = vis_speed + 1;
        end

        t = t + vis_speed;
    end
end
%% END of visualizeForwardDynamics.



function [pos, hTr_dp] = plotFKJointPos(fk_vqT, mkr, mkr_color)
    % get the new position (translation) of the computed forward kinematics ...
    %[p,~] = WBM.utilities.frame2posRotm(fk_vqT(1,1:7,idx)');
    %[p,~] = WBM.utilities.frame2posRotm(fk_vqT);
    p = fk_vqT(1:3,1);
    pos = p';
    % create a 3D data point of the forward kin. translation ...
    hTr_dp = plot3(pos(1,1), pos(1,2), pos(1,3), 'Marker', mkr, 'MarkerEdgeColor', mkr_color);
end

function [pos, hTr_dp] = updateFKJointPositions(hTr_dp, fk_vqT, nJnts)
    % update the forward kinematic translations (positions) ...
    for i = 1:nJnts
        % get the translation (position) of the current instance ...
        %[p,~] = WBM.utilities.frame2posRotm(fk_vqT);
        %p = fk_vqT(i,1:3);
        p = fk_vqT(1:3,i);
        pos = p';
        % update the position of the current 3D data-point ...
        set(hTr_dp(1,i), 'XData', pos(1,1), 'YData', pos(1,2), 'ZData', pos(1,3));
    end
end

function jnt_pair_pos = getLinkPositions(fwd_kin, jnt_pairs_idx, nLnks)
    jnt_pair_pos = zeros(nLnks,6);
    for i = 1:nLnks
        idx = jnt_pairs_idx(i,1:6);
        %                                               x1                       x2                       y1
        jnt_pair_pos(i,1:6) = horzcat( fwd_kin.vtr(idx(1,1),1), fwd_kin.vtr(idx(1,2),1), fwd_kin.vtr(idx(1,3),2), ...
                                       fwd_kin.vtr(idx(1,4),2), fwd_kin.vtr(idx(1,5),3), fwd_kin.vtr(idx(1,6),3) );
        %                                               y2                       z1                       z2
    end
end

function hull_vtx = getHullVertices(jnt_pair_pos, size_sf)
    % compute the hull vertices around the current link of the robot:
    % determine the orientation of the current link ...
    %                                 x2                  x1
    lnk_vec = horzcat( jnt_pair_pos(1,2) - jnt_pair_pos(1,1), ...
                       jnt_pair_pos(1,4) - jnt_pair_pos(1,3), ...
                       jnt_pair_pos(1,6) - jnt_pair_pos(1,5) );
    %                                 y2                  y1  
    %                                 z2                  z1  

    % calculate the orthonormal basis for the null space (kernel) of "lnk_vec" ...
    onb_lnk = null(lnk_vec); % orthonormal vectors to the link
    % scale the orthonormal vectors ...
    on_sc_vec_1 = size_sf(1,1) * onb_lnk(:,1);
    on_sc_vec_2 = size_sf(1,2) * onb_lnk(:,2);

    % calculate the offsets in the direction orthogonal to the link ...
    ofs = zeros(4,3);
    ofs(1,1:3) = on_sc_vec_1 + on_sc_vec_2;
    ofs(2,1:3) = -on_sc_vec_1 + on_sc_vec_2;
    ofs(3,1:3) = -on_sc_vec_1 - on_sc_vec_2;
    ofs(4,1:3) = on_sc_vec_1 - on_sc_vec_2;

    % compute the vertices of each patch which are around the link ...
    idx = 1;
    hull_vtx = zeros(8,3);
    for i = 2:-1:1
        for j = 1:4
            hull_vtx(idx,1:3) = horzcat( jnt_pair_pos(1,i)   + ofs(j,1), ...
                                         jnt_pair_pos(1,i+2) + ofs(j,2), ...
                                         jnt_pair_pos(1,i+4) + ofs(j,3) );
            idx = idx + 1;
        end
    end
end

function lnk_hull = createLinkHull(jnt_pair_pos, size_sf, faces)
    % compute the hull around the current link of the robot:
    hull_vtx = getHullVertices(jnt_pair_pos, size_sf);
    % create several filled polygons to form the hull of the link ...
    lnk_hull = patch('Vertices', hull_vtx, 'Faces', faces, 'FaceAlpha', 0.2);
end

function lnk_hull = updateLinkHullPos(lnk_hull, jnt_pair_pos, size_sf)
    % compute the new vertex positions for the hull of the current link ... 
    new_vtx_pos = getHullVertices(jnt_pair_pos, size_sf);
    % update the position of the hull ...
    set(lnk_hull, 'Vertices', new_vtx_pos);
end

function foot_vtx = getFootVertices(jnt_pair_pos, cf)
    % orthonormal vectors to the link ...
    on_vec_1 = [0 0.03 0]';
    on_vec_2 = [0 0 0.03]';

    % calculate the offsets in the direction orthogonal to the link ...
    ofs = zeros(4,3);
    ofs(1,1:3) = on_vec_1 + 2*on_vec_2;
    ofs(2,1:3) = -on_vec_1 + 2*on_vec_2;
    ofs(3,1:3) = -on_vec_1 - on_vec_2;
    ofs(4,1:3) = on_vec_1 - on_vec_2;

    % compute the vertices of each patch that forms the foot ...
    idx = 1;
    foot_vtx = zeros(8,3);
    for i = 1:2
        for j = 1:4
            foot_vtx(idx,1:3) = horzcat( jnt_pair_pos(1,2) + ofs(j,1) + cf(idx,1), ...
                                         jnt_pair_pos(1,4) + ofs(j,2) + cf(idx,2), ...
                                         jnt_pair_pos(1,6) + ofs(j,3) + cf(idx,3) );
            idx = idx + 1;
        end
    end
end

function foot_shape = createFootShape(jnt_pair_pos, cf, faces)
    % compute the shape of the current foot of the robot:
    foot_vtx = getFootVertices(jnt_pair_pos, cf);
    % create several filled polygons to form the shape of the foot ...
    foot_shape = patch('Vertices', foot_vtx, 'Faces', faces, 'FaceAlpha', 0.2);
end

function foot_shape = updateFootPos(foot_shape, jnt_pair_pos, cf)
    % compute the new vertex positions for the shape of the foot ... 
    new_vtx_pos = getFootVertices(jnt_pair_pos, cf);
    % update the position of the foot ...
    set(foot_shape, 'Vertices', new_vtx_pos);
end

function [hRobotBody, hLnk_lns] = createRobotBody(nLnks, jnt_pair_pos, nFeets, foot_jnts, foot_ds, hull_size_sf, hull_faces)
    %hLnk_lns = zeros(1,nLnks);
    hRobotBody = zeros(1,nLnks+nFeets);

    % plot the lines (edges) that are depicting the links to form the kinematic chain,
    % the skeleton, and the corresponding hull of each link of the robot:
    hLnk_lns = animatedline('LineWidth', 3, 'Color', 'k');
    for i = 1:nLnks
        % plot the skeleton, the link-edges, of the robot:
        %                   translation (x1,x2), translation (y1,y2), translation (z1,z2)
        addpoints(hLnk_lns, jnt_pair_pos(i,1:2), jnt_pair_pos(i,3:4), jnt_pair_pos(i,5:6));
        %addpoints(hLnk_lns(1,i), jnt_pair_pos(i,1:2), jnt_pair_pos(i,3:4), jnt_pair_pos(i,5:6));

        % create the hull, the shape, for the link  which is around the link line ...
        hRobotBody(1,i) = createLinkHull(jnt_pair_pos(i,1:6), hull_size_sf(i,1:2), hull_faces);
    end

    % plot the shapes of the feets:
    for i = 1:nFeets
        jnt_idx = foot_jnts(i);
        hRobotBody(1,nLnks+i) = createFootShape(jnt_pair_pos(jnt_idx,1:6), foot_ds, hull_faces);
    end
end

function [hRobotBody, hLnk_lns] = updateRobotBody(hRobotBody, hLnk_lns, nLnks, new_jnt_pair_pos, nFeets, foot_jnts, foot_ds, hull_size_sf)
    % update the link positions of the skeleton and the link hull positions of the robot:
    clearpoints(hLnk_lns); % remove all old point positions ...
    for i = 1:nLnks
        % update the link positions ...
        %                          new pos. (x1,x2),        new pos. (y1,y2),        new pos. (z1,z2)
        addpoints(hLnk_lns, new_jnt_pair_pos(i,1:2), new_jnt_pair_pos(i,3:4), new_jnt_pair_pos(i,5:6));
        % update the hull position of the current link ...
        hRobotBody(1,i) = updateLinkHullPos(hRobotBody(1,i), new_jnt_pair_pos(i,1:6), hull_size_sf(i,1:2));
    end

    % update the positions of the feets ...
    for i = 1:nFeets
        jnt_idx = foot_jnts(i);
        hRobotBody(1,nLnks+i) = updateFootPos(hRobotBody(1,nLnks+i), new_jnt_pair_pos(jnt_idx,1:6), foot_ds);
    end
end
