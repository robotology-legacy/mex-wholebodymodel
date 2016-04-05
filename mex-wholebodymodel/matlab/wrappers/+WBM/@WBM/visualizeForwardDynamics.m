function visualizeForwardDynamics(obj, x_out, tspan, sim_config, sim_tstep, vis_speed)
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

    obj.setWorldFrame2FixedLink(obj.mwbm_config.initStateParams.q_j, obj.mwbm_config.initStateParams.dq_j, ...
                                v_b, obj.mwbm_params.g_wf);

    nJnts = sim_config.robot_body.nJoints; % number of nodes (virtual joints) to be plotted
    nLnks = sim_config.robot_body.nLinks;  % number of edges (virtual links) to be plotted
    nFeets = sim_config.robot_body.nFeets; % number of feets of the robot's body

    %% Forward Kinematics of the robot's skeleton:

    % create a data structure for the forward kinematics + memory allocation:
    %
    % forward kinematic translation vector (xyz-position) of the joints:
    fwd_kin.vJntPos = zeros(nJnts,3);
    % 3D data-points of the forward kin. translations (positions) of the joints:
    fwd_kin.hJnt_dp = zeros(1,nJnts);
    % joint pair (link) translations (xyz-positions):
    fwd_kin.jnt_pair_pos = zeros(nLnks,6);
    % forward kin. roto-translation (in VQS-form):
    fwd_kin.vqT = zeros(nRes,7,nJnts);

    % calculate the forward kinematic roto-translation
    % of each joint of the robot's joint name list:

    %fwd_kin.vqT(1:nRes,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ... (to check!)
    for i = 1:nRes % for each result ...
        for j = 1:nJnts
            % convert the state of the base in roto-translation form ...
            [p_b, R_b] = WBM.utilities.frame2posRotm( squeeze(vqT_b(i,1:7)') );
            % set the world frame to the base ...
            obj.setWorldFrame(R_b, p_b, obj.mwbm_params.g_wf);
            % forward kinematic each joint (link) in the list ...
            fwd_kin.vqT(i,1:7,j) = (obj.forwardKinematics(sim_config.robot_body.joint_names{j,1}, R_b, p_b, q_j(i,1:ndof)'))';
        end
    end
    fwd_kin.vqT(1:nRes,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ...  (to check!)

    %% Initial plot of the animation environment:

    % clear and reset the subplots of the main figure in the reverse order:
    for i = 4:-1:1
        % set the current axes (subplot) of the main figure ...
        set(sim_config.hMainFigure, 'CurrentAxes', sim_config.hAxes(1,i));
        cla; % delete all graphics objects from the current axes
        axis off;
        % redraw the ground (rectangle) with a new color ...
        patch(sim_config.ground_shape(1,1:4), sim_config.ground_shape(2,1:4), ...
              sim_config.ground_shape(3,1:4), sim_config.robot_body.draw_prop.ground_color);
        % fill3(sim_config.ground_shape(1,1:4), sim_config.ground_shape(2,1:4), ...
        %       sim_config.ground_shape(3,1:4), sim_config.robot_body.draw_prop.ground_color);

        % draw the origin point of the axis onto the ground ...
        plot3(0, 0, 0, 'Marker', '.', 'MarkerSize', 3.5, 'MarkerEdgeColor', 'k');

        drawnow; % update and draw the current subplot in the main figure
    end
    
    %% Initial plot of the robot's body:

    % plot the initial base position of the robot's skeleton:
    % draw the joints of the robot skeleton ...
    plot_prop = sim_config.robot_body.draw_prop.joints;
    for i = 1:nJnts-1
        [fwd_kin.vJntPos(i,1:3), fwd_kin.hJnt_dp(1,i)] = plotFKJointPos((fwd_kin.vqT(1,1:7,i))', plot_prop);
    end
    % draw the position of the center of mass (CoM) ...
    plot_prop = sim_config.robot_body.draw_prop.com;    
    [fwd_kin.vJntPos(nJnts,1:3), fwd_kin.hJnt_dp(1,nJnts)] = plotFKJointPos((fwd_kin.vqT(1,1:7,nJnts))', plot_prop);

    % create a position parameter matrix from the defined joint pairs (links) to describe a full
    % configuration of the robot's skeleton:
    fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin, sim_config.robot_body.joint_pair_idx{1,1}, nLnks);

    % create some property structures for drawing the links and the hull of the robot ...
    hull_geom = sim_config.robot_body.hull_geometry;
    foot_geom = sim_config.robot_body.foot_geometry;

    lnk_draw_prop = sim_config.robot_body.draw_prop.links;
    hull_draw_prop = sim_config.robot_body.draw_prop.hull;

    % create the body of the robot to be animated:
    [body.hShape, body.hLnk_lns] = createRobotBody(nLnks, fwd_kin.jnt_pair_pos, nFeets, foot_geom, ...
                                                   hull_geom, lnk_draw_prop, hull_draw_prop);

    % delete all plot objects of all the subplots (nr. 1, 2, 3 and 4) in the main figure ...
    if ~isempty(sim_config.plot_objs)
        for i = 1:4
            delete(sim_config.plot_objs{1,i});
        end
    end

    % store all axes handle objects into a graphics object array ...
    nHullElem = nLnks + nFeets;
    nGObjs    = nJnts + nLnks + nHullElem;
    idx1 = nJnts + 1;
    idx2 = nJnts + nLnks;
    idx3 = idx2 + 1;

    hSimRobot = gobjects(nGObjs,1);
    hSimRobot(1:nJnts,1)     = fwd_kin.hJnt_dp(1,1:nJnts);
    hSimRobot(idx1:idx2,1)   = body.hLnk_lns(1,1:nLnks);
    hSimRobot(idx3:nGObjs,1) = body.hShape(1,1:nHullElem);
        
    % shows the simulated robot in perspective view (left bottom of the frame) ...
    sim_config.plot_objs{1,1} = hSimRobot;
    
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

    %% Update the graphic objects of the robot's body:
    %
    t = 2;
    while (t <= nRes) % the visualization instance ...
        tic; % visualization step timer start (needed for adapting the visualization speed)

        % update the forward kinematic translations (positions) ...
        new_fk_pos = (squeeze(fwd_kin.vqT(t,1:7,1:nJnts)))';
        [fwd_kin.vJntPos, fwd_kin.hJnt_dp] = updateFKJointPositions(fwd_kin.vJntPos, fwd_kin.hJnt_dp, new_fk_pos, nJnts);

        % update the position parameter matrix for the links of the robot with the 2nd joint pair list ...
        fwd_kin.jnt_pair_pos = getLinkPositions(fwd_kin, sim_config.robot_body.joint_pair_idx{2,1}, nLnks);

        % update the edges of the skeleton with the respect to the new joint positions ...
        [body.hShape, body.hLnk_lns] = updateRobotBody(body.hShape, body.hLnk_lns, nLnks, fwd_kin.jnt_pair_pos, ...
                                                       nFeets, foot_geom, hull_geom.size_sf);

        % replace all graphics objects in the array with the new graphic objects ...
        hSimRobot(1:nJnts,1)     = fwd_kin.hJnt_dp(1,1:nJnts);
        hSimRobot(idx1:idx2,1)   = body.hLnk_lns(1,1:nLnks);
        hSimRobot(idx3:nGObjs,1) = body.hShape(1,1:nHullElem);

        sim_config.plot_objs{1,1} = hSimRobot;

        % delete the old graphic objects and copy the updated array of new plot objects into the
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



function [pos, hJnt_dp] = plotFKJointPos(fk_vqT, plot_prop)
    % get the new position (translation) of the computed forward kinematics ...
    p = fk_vqT(1:3,1);
    pos = p';
    % create a 3D data point of the forward kin. translation ...
    hJnt_dp = plot3(pos(1,1), pos(1,2), pos(1,3), 'Marker', plot_prop.marker, ...
                    'MarkerSize', plot_prop.marker_sz, 'MarkerEdgeColor', plot_prop.color);
end

function [pos, hJnt_dp] = updateFKJointPositions(pos, hJnt_dp, fk_vqT, nJnts)
    % update the forward kinematic translations (positions) ...
    for i = 1:nJnts
        % get the translation (position) of the current instance ...
        p = fk_vqT(i,1:3);
        pos(i,1:3) = p;
        % update the position of the current 3D data-point ...
        set(hJnt_dp(1,i), 'XData', pos(i,1), 'YData', pos(i,2), 'ZData', pos(i,3));
    end
end

function jnt_pair_pos = getLinkPositions(fwd_kin, jnt_pairs_idx, nLnks)
    jnt_pair_pos = zeros(nLnks,6);
    for i = 1:nLnks
        idx = jnt_pairs_idx(i,1:6);
        %                                                   x1                           x2                           y1
        jnt_pair_pos(i,1:6) = horzcat( fwd_kin.vJntPos(idx(1,1),1), fwd_kin.vJntPos(idx(1,2),1), fwd_kin.vJntPos(idx(1,3),2), ...
                                       fwd_kin.vJntPos(idx(1,4),2), fwd_kin.vJntPos(idx(1,5),3), fwd_kin.vJntPos(idx(1,6),3) );
        %                                                   y2                           z1                           z2
    end
end

function hull_vtx = getHullVertices(jnt_pair_pos, hull_size_sf)
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
    on_sc_vec_1 = hull_size_sf(1,1) * onb_lnk(:,1);
    on_sc_vec_2 = hull_size_sf(1,2) * onb_lnk(:,2);

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

function lnk_hull = createLinkHull(jnt_pair_pos, hull_size_sf, hull_faces, draw_prop)
    % compute the hull around the current link of the robot:
    hull_vtx = getHullVertices(jnt_pair_pos, hull_size_sf);
    % create several filled polygons to form the hull of the link ...
    lnk_hull = patch('Vertices', hull_vtx, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                     'Faces', hull_faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function lnk_hull = updateLinkHullPos(lnk_hull, jnt_pair_pos, hull_size_sf)
    % compute the new vertex positions for the hull of the current link ... 
    new_vtx_pos = getHullVertices(jnt_pair_pos, hull_size_sf);
    % update the position of the hull ...
    set(lnk_hull, 'Vertices', new_vtx_pos);
end

function foot_vtx = getFootVertices(jnt_pair_pos, base_sz, foot_ds)
    % orthonormal vectors to the link ...
    on_vec_1 = [0 base_sz.width              0]';
    on_vec_2 = [0 0             base_sz.height]';

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
            foot_vtx(idx,1:3) = horzcat( jnt_pair_pos(1,2) + ofs(j,1) + foot_ds(idx,1), ...
                                         jnt_pair_pos(1,4) + ofs(j,2) + foot_ds(idx,2), ...
                                         jnt_pair_pos(1,6) + ofs(j,3) + foot_ds(idx,3) );
            idx = idx + 1;
        end
    end
end

function foot_shape = createFootShape(jnt_pair_pos, base_sz, foot_ds, hull_faces, draw_prop)
    % compute the shape of the current foot of the robot:
    foot_vtx = getFootVertices(jnt_pair_pos, base_sz, foot_ds);
    % create several filled polygons to form the shape of the foot ...
    foot_shape = patch('Vertices', foot_vtx, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                       'Faces', hull_faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function foot_shape = updateFootPos(foot_shape, jnt_pair_pos, base_sz, foot_ds)
    % compute the new vertex positions for the shape of the foot ... 
    new_vtx_pos = getFootVertices(jnt_pair_pos, base_sz, foot_ds);
    % update the position of the foot ...
    set(foot_shape, 'Vertices', new_vtx_pos);
end

function [hRobotBody, hLnk_lns] = createRobotBody(nLnks, jnt_pair_pos, nFeets, foot_geom, hull_geom, lnk_draw_prop, hull_draw_prop)
    % draw the lines (edges) that are depicting the links to form the kinematic chain,
    % the skeleton, and the corresponding hull of each link of the robot:
    hLnk_lns   = gobjects(1,nLnks);
    hRobotBody = zeros(1,nLnks+nFeets);
    for i = 1:nLnks
        hLnk_lns(1,i) = animatedline('LineWidth', lnk_draw_prop.line_width, 'Color', lnk_draw_prop.color);
        % draw the skeleton, the link-edges, of the robot:
        %                        translation (x1,x2), translation (y1,y2), translation (z1,z2)
        addpoints(hLnk_lns(1,i), jnt_pair_pos(i,1:2), jnt_pair_pos(i,3:4), jnt_pair_pos(i,5:6));
        
        % create the hull, the shape, for the link  which is around the link line ...
        hRobotBody(1,i) = createLinkHull(jnt_pair_pos(i,1:6), hull_geom.size_sf(i,1:2), hull_geom.faces, hull_draw_prop);
    end

    % draw the shapes of the feets:
    for i = 1:nFeets
        jnt_idx = foot_geom.joints(i);
        hRobotBody(1,nLnks+i) = createFootShape(jnt_pair_pos(jnt_idx,1:6), foot_geom.base_sz, foot_geom.shape_ds, ...
                                                hull_geom.faces, hull_draw_prop);
    end
end

function [hRobotBody, hLnk_lns] = updateRobotBody(hRobotBody, hLnk_lns, nLnks, new_jnt_pair_pos, nFeets, foot_geom, hull_size_sf)
    % update the link positions of the skeleton and the link hull positions of the robot:
    for i = 1:nLnks
        % remove the old point positions ...
        clearpoints(hLnk_lns(1,i)); 
        % update the link positions ...
        %                               new pos. (x1,x2),        new pos. (y1,y2),        new pos. (z1,z2)
        addpoints(hLnk_lns(1,i), new_jnt_pair_pos(i,1:2), new_jnt_pair_pos(i,3:4), new_jnt_pair_pos(i,5:6));
        % update the hull position of the current link ...
        hRobotBody(1,i) = updateLinkHullPos(hRobotBody(1,i), new_jnt_pair_pos(i,1:6), hull_size_sf(i,1:2));
    end

    % update the positions of the feets ...
    for i = 1:nFeets
        jnt_idx = foot_geom.joints(i);
        hRobotBody(1,nLnks+i) = updateFootPos(hRobotBody(1,nLnks+i), new_jnt_pair_pos(jnt_idx,1:6), ...
                                              foot_geom.base_sz, foot_geom.shape_ds);
    end
end
