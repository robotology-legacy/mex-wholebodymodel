function visualizeForwardDynamics(obj, x_out, sim_config, sim_tstep, vis_ctrl)
    if ~exist('vis_ctrl', 'var')
        % draw all graphic elements of the simulated robot ...
        vis_ctrl.drawJnts  = true;
        vis_ctrl.drawCom   = true;
        vis_ctrl.drawSkel  = true;
        vis_ctrl.drawBody  = true;
        % use the default visualization speed value ...
        % (note: use another speed value in order to make the visualization speed
        %        close to real-time when the simulation time step is changed.)
        vis_speed = 1.0;
    elseif isstruct(vis_ctrl)
        vis_speed = vis_ctrl.vis_speed;
    else
        error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
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

    obj.setWorldFrame2FixedLink(obj.mwbm_config.initStateParams.q_j, obj.mwbm_config.initStateParams.dq_j, ...
                                v_b, obj.mwbm_params.g_wf);

    nJnts  = sim_config.robot_body.nJoints; % number of nodes (virtual joints) to be plotted
    nLnks  = sim_config.robot_body.nLinks;  % number of edges (virtual links) to be plotted
    nFeets = sim_config.robot_body.nFeets;  % number of feets of the robot's body

    %% Forward Kinematics of the robot's skeleton (joints):

    % create a data structure for the forward kinematics + memory allocation:
    %
    % forward kinematic translation vector (xyz-position) of the joints:
    fwd_kin.vJntPos = zeros(nJnts,3);
    % 3D data-points of the forward kin. translations (positions) of the joints:
    fwd_kin.hJnt_dp = zeros(1,nJnts);
    % joint pair (link) translations (xyz-positions):
    fwd_kin.jnt_pair_pos = zeros(nLnks,6);
    % forward kin. roto-translation (in VQS-form):
    fwd_kin_vqT = zeros(nRes,7,nJnts);

    % calculate the forward kinematic roto-translation of each joint in the
    % joint name list of the robot:
    fwd_kin.vqT(1:nRes,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ...
    for i = 1:nRes % for each result ...
        q   = q_j(i,1:ndof)';
        vqT = squeeze(vqT_b(i,1:7)');

        for j = 2:nJnts
            fwd_kin.vqT(i,1:7,j) = obj.computeFKinRotoTranslation(sim_config.robot_body.joint_names{j,1}, q, vqT);
        end
    end

    %% Initial plot of the animation environment:

    % reset all subplots in the main figure and reinitialize them with a new ground floor ...
    initSimEnvironment(sim_config);

    %% Initial plot of the robot in base position:

    nGObjs    = nJnts;                    % number of graphic objects to be drawn
    maxNGObjs = nJnts + 2*nLnks + nFeets; % max. number of graphics objects
    hSimRobot = gobjects(maxNGObjs,1);    % graphic objects array for the animated robot

    % draw the data points for the joints of the robot's skeleton:
    plot_prop = sim_config.robot_body.draw_prop.joints;
    if ~vis_ctrl.drawJnts
        plot_prop.marker = 'none';
        plot_prop.color  = 'none';
    end

    for i = 1:nJnts-1
        [fwd_kin.vJntPos(i,1:3), fwd_kin.hJnt_dp(1,i)] = plotFKinJointPos((fwd_kin.vqT(1,1:7,i))', plot_prop);
    end
    % draw the position of the center of mass (CoM):
    plot_prop = sim_config.robot_body.draw_prop.com;
    if ~vis_ctrl.drawCom
        plot_prop.marker = 'none';
        plot_prop.color  = 'none';
    end
    [fwd_kin.vJntPos(nJnts,1:3), fwd_kin.hJnt_dp(1,nJnts)] = plotFKinJointPos((fwd_kin.vqT(1,1:7,nJnts))', plot_prop);
    hSimRobot(1:nGObjs,1) = fwd_kin.hJnt_dp(1,1:nJnts); % store the graphics objects ...

    % create a position parameter matrix from the defined joint pairs (links) to describe a full
    % configuration of the robot's skeleton:
    fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin.vJntPos, sim_config.robot_body.joint_pair_idx, nLnks);

    if vis_ctrl.drawSkel
        % draw the skeleton of the robot:
        lnk_draw_prop = sim_config.robot_body.draw_prop.links;
        body.hLnk_lns = createRobotSkeleton(nLnks, fwd_kin.jnt_pair_pos, lnk_draw_prop);

        idx1   = nJnts + 1;
        idx2   = nJnts + nLnks;
        nGObjs = idx2; % update the number of graphic objects ...
        hSimRobot(idx1:nGObjs,1) = body.hLnk_lns(1,1:nLnks); % add the g-objects to the array ...
    end

    if vis_ctrl.drawBody
        % create the shape for the animated body of the robot:
        % geometry and draw properties ...
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;
        shp_draw_prop = sim_config.robot_body.draw_prop.shape;

        body.hShape = createRobotBody(nLnks, fwd_kin.jnt_pair_pos, nFeets, foot_geom, shape_geom, shp_draw_prop);

        nShpElem = nLnks + nFeets;
        idx3     = nGObjs + 1;
        nGObjs   = nGObjs + nShpElem; % update obj-number ...
        hSimRobot(idx3:nGObjs,1) = body.hShape(1,1:nShpElem); % store the g-objects ...
    end

    % delete all plot objects of all the subplots (nr. 1, 2, 3 and 4) in the main figure ...
    if ~isempty(sim_config.plot_objs)
        for i = 1:4
            delete(sim_config.plot_objs{1,i});
        end
    end

    % put all graphics objects into the axes of the first subplot of the frame (left bottom):
    % (the simulated robot will be shown in perspective view)
    sim_config.plot_objs{1,1} = hSimRobot(1:nGObjs,1);

    % copy all objects to the other axes with different views:
    % view top:
    set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,2));
    sim_config.plot_objs{1,2} = copyobj(sim_config.plot_objs{1,1}, sim_config.hAxes(1,2));
    view(-90,90); % change the viewpoint (azimuth, elevation)
    % view side:
    set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,3));
    sim_config.plot_objs{1,3} = copyobj(sim_config.plot_objs{1,1}, sim_config.hAxes(1,3));
    view(0,1);
    % view front:
    set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,4));
    sim_config.plot_objs{1,4} = copyobj(sim_config.plot_objs{1,1}, sim_config.hAxes(1,4));
    view(-90,1);
    % set back the current axes handle to the first subplot ...
    set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,1));

    drawnow; % draw all plots of the initial robot body in the main figure ...

    %% Update the graphic objects of the robot:

    t = 2;
    while (t <= nRes) % the visualization instance ...
        tic; % visualization step timer start (needed for adapting the visualization speed)

        % update the forward kinematic translations (positions) ...
        new_fk_pos = (squeeze(fwd_kin.vqT(t,1:7,1:nJnts)))';
        [fwd_kin.vJntPos, fwd_kin.hJnt_dp] = updateFKinJointPositions(fwd_kin.vJntPos, fwd_kin.hJnt_dp, new_fk_pos, nJnts);
        hSimRobot(1:nJnts,1) = fwd_kin.hJnt_dp(1,1:nJnts);

        % update the position parameter matrix for the links of the robot ...
        fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin.vJntPos, sim_config.robot_body.joint_pair_idx, nLnks);

        if vis_ctrl.drawSkel
            % update the edges (links) of the skeleton with the respect to the new joint positions ...
            body.hLnk_lns = updateRobotSkeleton(body.hLnk_lns, nLnks, fwd_kin.jnt_pair_pos);
            hSimRobot(idx1:idx2,1) = body.hLnk_lns(1,1:nLnks);
        end

        if vis_ctrl.drawBody
            % update the shape positions for the corresponding links of the robot ...
            body.hShape = updateRobotBody(body.hShape, nLnks, fwd_kin.jnt_pair_pos, nFeets, foot_geom, shape_geom.size_sf);
            hSimRobot(idx3:nGObjs,1) = body.hShape(1,1:nShpElem);
        end

        % update the graphics objects in the first subplot ...
        sim_config.plot_objs{1,1} = hSimRobot(1:nGObjs,1);

        % delete the old graphic objects and copy the updated array of the new plot objects into the
        % specified cell. The new graphics elements are children of the current axes handle (subplots):
        for i = 2:4
            delete(sim_config.plot_objs{1,i});
            sim_config.plot_objs{1,i} = copyobj(sim_config.plot_objs{1,1}, sim_config.hAxes(1,i));
        end
        drawnow;

        % update the visualization speed to keep the simulation very close to the real time ...
        tdiff = vis_speed * sim_tstep - toc();
        if (tdiff > 0)
            % make a short break ...
            pause(tdiff);
        else
            % increase the visualization speed by 1 ...
            vis_speed = vis_speed + 1;
        end
        t = t + vis_speed;
    end
end
%% END of visualizeForwardDynamics.



%% GRAPHIC FUNCTIONS:

function initSimEnvironment(sim_config)
    % clear and reset the subplots of the main figure in the reverse order ...
    for i = 4:-1:1
        % set the axes of the current subplot in the main figure ...
        set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,i));
        cla; % delete all graphics objects from the current axes
        axis off;
        % redraw the ground (rectangle) with a new color ...
        fill3(sim_config.environment.ground_shape(1,1:4), sim_config.environment.ground_shape(2,1:4), ...
              sim_config.environment.ground_shape(3,1:4), sim_config.environment.ground_color, ...
              'EdgeColor', sim_config.environment.ground_edge_color);
        % draw the origin point of the axis onto the ground ...
        plot3(0, 0, 0, 'Marker', '.', 'MarkerSize', sim_config.environment.origin_pt_size, ...
              'MarkerEdgeColor', sim_config.environment.origin_pt_color);

        drawnow; % update and draw the current subplot in the main figure
    end
end

function [jntPos, hJnt_dp] = plotFKinJointPos(fk_vqT, plot_prop)
    % get the new position (translation) of the computed forward kinematics ...
    p = fk_vqT(1:3,1);
    jntPos = p';
    % create a 3D data point of the forward kin. translation ...
    hJnt_dp = plot3(jntPos(1,1), jntPos(1,2), jntPos(1,3), 'Marker', plot_prop.marker, ...
                    'MarkerSize', plot_prop.marker_sz, 'MarkerEdgeColor', plot_prop.color);
end

function [vJntPos, hJnt_dp] = updateFKinJointPositions(vJntPos, hJnt_dp, fk_vqT, nJnts)
    % update the forward kinematic translations (positions) ...
    for i = 1:nJnts
        % get the translation (position) of the current instance ...
        p = fk_vqT(i,1:3);
        vJntPos(i,1:3) = p;
        % update the position of the current 3D data-point ...
        set(hJnt_dp(1,i), 'XData', vJntPos(i,1), 'YData', vJntPos(i,2), 'ZData', vJntPos(i,3));
    end
end

function jnt_pair_pos = getLinkPositions(vJntPos, jnt_pairs_idx, nLnks)
    % compute the position parameter matrix for the links (joint pairs) of the
    % robot with a predefined joint pair list:
    jnt_pair_pos = zeros(nLnks,6);
    for i = 1:nLnks
        idx = jnt_pairs_idx(i,1:6);
        %                                           x1                   x2                   y1
        jnt_pair_pos(i,1:6) = horzcat( vJntPos(idx(1,1),1), vJntPos(idx(1,2),1), vJntPos(idx(1,3),2), ...
                                       vJntPos(idx(1,4),2), vJntPos(idx(1,5),3), vJntPos(idx(1,6),3) );
        %                                           y2                   z1                   z2
    end
end

function shape_vtx = getLnkShapeVertices(jnt_pair_pos, size_sf)
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
    shape_vtx = zeros(8,3);
    for i = 2:-1:1
        for j = 1:4
            shape_vtx(idx,1:3) = horzcat( jnt_pair_pos(1,i)   + ofs(j,1), ...
                                          jnt_pair_pos(1,i+2) + ofs(j,2), ...
                                          jnt_pair_pos(1,i+4) + ofs(j,3) );
            idx = idx + 1;
        end
    end
end

function lnk_shape = createLinkShape(jnt_pair_pos, size_sf, faces, draw_prop)
    % compute a simple hull (box shape) around the current link of the robot:
    shape_vtx = getLnkShapeVertices(jnt_pair_pos, size_sf);
    % create several filled polygons to form the hull (box shape) for the current link ...
    lnk_shape = patch('Vertices', shape_vtx, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                      'Faces', faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function lnk_shape = updateLinkShapePos(lnk_shape, jnt_pair_pos, size_sf)
    % compute the new vertex positions for the box shape of the current link ...
    new_vtx_pos = getLnkShapeVertices(jnt_pair_pos, size_sf);
    % update the position of the shape ...
    set(lnk_shape, 'Vertices', new_vtx_pos);
end

function foot_vtx = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds)
    % scaled orthogonal (lin. independent) vectors to the current foot joint ...
    o_vec_1 = [0 base_sz.width              0]';
    o_vec_2 = [0 0             base_sz.height]';

    % calculate the offsets in the direction orthogonal to the foot joint ...
    ofs = zeros(4,3);
    ofs(1,1:3) = o_vec_1 + 2*o_vec_2;
    ofs(2,1:3) = -o_vec_1 + 2*o_vec_2;
    ofs(3,1:3) = -o_vec_1 - o_vec_2;
    ofs(4,1:3) = o_vec_1 - o_vec_2;

    % compute the vertices of each patch to form the foot shape ...
    idx = 1;
    foot_vtx = zeros(8,3);
    for i = 1:2 % while (idx <= 8)
        for j = 1:4
            foot_vtx(idx,1:3) = horzcat( jnt_pair_pos(1,2) + ofs(j,1) + foot_ds(idx,1), ...
                                         jnt_pair_pos(1,4) + ofs(j,2) + foot_ds(idx,2), ...
                                         jnt_pair_pos(1,6) + ofs(j,3) + foot_ds(idx,3) );
            idx = idx + 1;
        end
    end
end

function foot_shp = createFootShape(jnt_pair_pos, base_sz, foot_ds, faces, draw_prop)
    % compute the vertex positions to build the shape of the current foot of the robot ...
    foot_vtx = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds);
    % create several filled polygons to form the shape of the foot ...
    foot_shp = patch('Vertices', foot_vtx, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                     'Faces', faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function foot_shp = updateFootPos(foot_shp, jnt_pair_pos, base_sz, foot_ds)
    % compute the new vertex positions for the shape of the foot ...
    new_vtx_pos = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds);
    % update the position of the foot ...
    set(foot_shp, 'Vertices', new_vtx_pos);
end

function hLnk_lns = createRobotSkeleton(nLnks, jnt_pair_pos, draw_prop)
    % draw the lines (edges) that are depicting the links to form the
    % kinematic chain, the skeleton of the robot:
    hLnk_lns   = gobjects(1,nLnks);
    for i = 1:nLnks
        hLnk_lns(1,i) = animatedline('LineWidth', draw_prop.line_width, 'Color', draw_prop.color);
        % draw the skeleton, the link-edges, of the robot:
        %                        translation (x1,x2), translation (y1,y2), translation (z1,z2)
        addpoints(hLnk_lns(1,i), jnt_pair_pos(i,1:2), jnt_pair_pos(i,3:4), jnt_pair_pos(i,5:6));
    end
end

function hRobotShape = createRobotBody(nLnks, jnt_pair_pos, nFeets, foot_geom, shape_geom, draw_prop)
    % draw the box shapes (hull) for the corresponding links of the robot:
    hRobotShape = zeros(1,nLnks+nFeets);
    for i = 1:nLnks
        % create the shape of the link, which is around the link line ...
        hRobotShape(1,i) = createLinkShape(jnt_pair_pos(i,1:6), shape_geom.size_sf(i,1:2), shape_geom.faces, draw_prop);
    end

    % draw the foot shapes ...
    for i = 1:nFeets
        jnt_idx = foot_geom.joints(i);
        hRobotShape(1,nLnks+i) = createFootShape(jnt_pair_pos(jnt_idx,1:6), foot_geom.base_sz, foot_geom.shape_ds, ...
                                                 shape_geom.faces, draw_prop);
    end
end

function hLnk_lns = updateRobotSkeleton(hLnk_lns, nLnks, new_jnt_pair_pos)
    % update the link positions of the robot's skeleton:
    for i = 1:nLnks
        % remove the old point positions ...
        clearpoints(hLnk_lns(1,i));
        % update the link positions ...
        %                               new pos. (x1,x2),        new pos. (y1,y2),        new pos. (z1,z2)
        addpoints(hLnk_lns(1,i), new_jnt_pair_pos(i,1:2), new_jnt_pair_pos(i,3:4), new_jnt_pair_pos(i,5:6));
    end
end

function hRobotShape = updateRobotBody(hRobotShape, nLnks, new_jnt_pair_pos, nFeets, foot_geom, size_sf)
    % update the shape positions of the corresponding links of the robot:
    for i = 1:nLnks
        hRobotShape(1,i) = updateLinkShapePos(hRobotShape(1,i), new_jnt_pair_pos(i,1:6), size_sf(i,1:2));
    end

    % update the positions of the feets ...
    for i = 1:nFeets
        jnt_idx = foot_geom.joints(i);
        hRobotShape(1,nLnks+i) = updateFootPos(hRobotShape(1,nLnks+i), new_jnt_pair_pos(jnt_idx,1:6), ...
                                               foot_geom.base_sz, foot_geom.shape_ds);
    end
end
