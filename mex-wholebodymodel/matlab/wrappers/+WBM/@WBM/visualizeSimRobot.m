function visualizeSimRobot(obj, stmPos, sim_config, sim_tstep, vis_ctrl)
    % check the dimension and get the number of instances of the simulation result ...
    ndof = obj.mwbm_model.ndof;
    vlen = ndof + 7;
    [nSteps, len] = size(stmPos);
    if (len ~= vlen)
        error('WBM::visualizeSimRobot: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % get the translations, orientations and joint positions from the output vector
    % "stmPos" of the integration part of the forward dynamics function:
    vqT_b = stmPos(1:nSteps,1:7);
    q_j   = stmPos(1:nSteps,8:vlen);
    q_j   = q_j.';

    nJnts = sim_config.robot_body.nJnts; % number of nodes (virtual joints) to be plotted
    nLnks = sim_config.robot_body.nLnks; % number of edges (virtual links) to be plotted

    % create a data structure for the forward kinematics of
    % the robot's skeleton (joint positions/orientations):
    fkin_jnts.pos      = zeros(nJnts,3);        % fkin-translation vector of the joints
    fkin_jnts.pair_pos = zeros(nLnks,6);        % joint pair translations of the links (link end-nodes)
    fkin_jnts.vqT      = zeros(nSteps,7,nJnts); % fkin. VQ-transformation of the joints

    % calculate for each time step the forward kinematics (VQ-transformations)
    % of each joint of the robot's joint name list:
    fkin_jnts.vqT(1:nSteps,1:7,1) = vqT_b; % use the base data instead the forward kin. of the 'root_link' ...
    for i = 1:nSteps
        q   = q_j(1:ndof,i);
        vqT = squeeze(vqT_b(i,1:7)).';

        for j = 2:nJnts
            fkin_jnts.vqT(i,1:7,j) = fkinVQTransformation(obj, sim_config.robot_body.jnt_lnk_names{j,1}, q, vqT);
        end
    end

    if sim_config.mkvideo
        % show and animate the robot (update at each time step all positions of the
        % graphic objects) and return a set of video frames of all iteration steps:
        if ~isempty(sim_config.pl_stack)
            % with payload:
            % get for each time step the positions and orientations (frames) of the payload objects ...
            [vqT_vb, vb_idx, nVBds] = getPayloadFrames(obj, vqT_b, q_j, sim_config, nSteps);
            vidfrms = makeSimRobotVideoPL(fkin_jnts, vqT_vb, vb_idx, nVBds, sim_config, nSteps, vis_ctrl);
        else
            % without payload:
            vidfrms = makeSimRobotVideo(fkin_jnts, sim_config, nSteps, vis_ctrl);
        end
        vid = VideoWriter(sim_config.vid_filename, 'Uncompressed AVI');
        vid.FrameRate = sim_config.vid_fps;

        open(vid);
        writeVideo(vid, vidfrms);
        close(vid);
        return
    end
    % else, show only the robot animation:
    if ~isempty(sim_config.pl_stack)
        % with payload:
        [vqT_vb, vb_idx, nVBds] = getPayloadFrames(obj, vqT_b, q_j, sim_config, nSteps);
        animateSimRobotPL(fkin_jnts, vqT_vb, vb_idx, nVBds, sim_config, sim_tstep, nSteps, vis_ctrl);
        return
    end
    % without payload ...
    animateSimRobot(fkin_jnts, sim_config, sim_tstep, nSteps, vis_ctrl);
end
%% END of visualizeSimRobot.


%% DRAW GRAPHICS & ANIMATION FUNCTIONS:

function [vqT_vb, vb_idx, nVBds] = getPayloadFrames(obj, vqT_b, q_j, sim_config, nSteps)
    ndof  = obj.mwbm_model.ndof;
    nVBds = size(sim_config.pl_stack,1);

    vb_idx = zeros(nVBds,1);
    vqT_vb = zeros(7*nVBds,nSteps);

    i = -6;
    for j = 1:nVBds
        vb_idx(j,1) = sim_config.pl_stack{j,1};
        manip       = sim_config.pl_stack{j,2};
        switch manip
            case {'lh', 'bh'}
                % left hand or both hands:
                pl_idx = 1;
            case 'rh'
                % right hand:
                pl_idx = 2;
            otherwise
                error('getPayloadFrames: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
        end
        % get the utilization time indices:
        ti_s = sim_config.pl_time_idx(j,1); % start
        ti_e = sim_config.pl_time_idx(j,2); % end

        i  = i + 7;
        ci = i:7*j; % column indices

        idx1 = ti_s - 1;
        if (idx1 >= 1)
            % set the initial frame of the object's CoM before the obj. is grabbed
            % (i.e. time indices where the initial CoM doesn't move):
            vqT_init = sim_config.environment.vb_objects(j,1).frame;
            vqT_vb(ci,1:idx1) = repmat(vqT_init, 1, idx1);
        end

        % set the frames of the object while it was taken and manipulated with the hand(s) ...
        for k = ti_s:ti_e
            q   = q_j(1:ndof,k);
            vqT = squeeze(vqT_b(k,1:7)).';
            [wf_p_b, wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT);

            wf_H_cm = payloadFrame(obj, wf_R_b, wf_p_b, q, pl_idx);
            vqT_vb(ci,k) = WBM.utilities.tfms.tform2frame(wf_H_cm);
        end

        idx2 = ti_e + 1;
        if (idx2 <= nSteps)
            % set the final frame of the object's CoM after releasing the object
            % (time indices of the new final position of the object's CoM):
            vqT_vb(ci,idx2:nSteps) = repmat(vqT_vb(ci,ti_e), 1, nSteps-ti_e);
        end
    end
end

function [sim_config, hSimRobot, fkin_jnts, nRGObj, vidfrm] = createSimRobot(sim_config, fkin_jnts, vis_ctrl)
    gobjs_0 = gobjects(0);
    hLnkLines  = gobjs_0;
    hLnkShapes = gobjs_0;

    nLnks = sim_config.robot_body.nLnks;
    nFeet = sim_config.robot_body.nFeet; % number of feet of the robot's body

    % draw the joint nodes (data points) of the robot's skeleton and
    % the position of the the center of mass (CoM):
    [hJntNodes, fkin_jnts] = createJointNodes(fkin_jnts, sim_config, vis_ctrl);

    % create a position parameter matrix from the defined joint pairs (links) to describe a full
    % configuration of the robot's skeleton:
    fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, sim_config.robot_body.jnt_pair_idx, nLnks);

    if vis_ctrl.drawSkel
        % draw the skeleton (lines between the joints) of the robot:
        draw_prop_lnk = sim_config.robot_body.draw_prop.links;
        hLnkLines = createRobotSkeleton(fkin_jnts.pair_pos, nLnks, draw_prop_lnk);
    end

    if vis_ctrl.drawBody
        % create the shape for the animated body of the robot:
        % geometry and draw properties ...
        shape_geom    = sim_config.robot_body.shape_geom;
        foot_geom     = sim_config.robot_body.foot_geom;
        draw_prop_shp = sim_config.robot_body.draw_prop.shape;

        hLnkShapes = createRobotBody(fkin_jnts.pair_pos, nLnks, nFeet, foot_geom, shape_geom, draw_prop_shp);
    end

    hSimRobot = vertcat(hJntNodes, hLnkLines, hLnkShapes);
    nRGObj    = size(hSimRobot,1); % number of graphic objects of the robot to be drawn

    % draw the robot in all subplot positions (axes) of the figure window ...
    sim_config = drawSimRobotAxes(sim_config, hSimRobot);

    if (nargout == 5)
        if (sim_config.vid_axes_idx > 0)
            % capture only the specified subplot frame ...
            set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,sim_config.vid_axes_idx));
            vidfrm = getframe(gca);
            return
        end
        % else, capture the entire figure window ...
        vidfrm = getframe(sim_config.hWndFigure);
    end
end

function sim_config = drawSimRobotAxes(sim_config, hSimRobot)
    if ~iscellstr(sim_config.axes_views)
        error('drawSimRobotAxes: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    nAxes = sim_config.nAxes;
    % put all graphics objects into the axes of the first
    % subplot of the figure window (left bottom if nAxes > 2):
    sim_config.gfx_objects{1,1} = vertcat(hSimRobot, sim_config.gfx_objects{1,1});
    setView(sim_config, 1);
    if (nAxes > 1)
        for i = 2:nAxes
            set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,i));
            sim_config.gfx_objects{1,i} = copyobj(sim_config.gfx_objects{1,1}, sim_config.hAxes(1,i));
            setView(sim_config, i);
        end
        % set back the current axes handle to the first subplot ...
        set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,1));
    end
    drawnow; % draw all plots of the initial robot body in the main figure ...
end

function setView(sim_config, ax_idx)
    idx = find(strcmp(sim_config.axes_vwpts(:,1), sim_config.axes_views{1,ax_idx}));
    if isempty(idx)
        error('setView: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
    end
    view(sim_config.axes_vwpts{idx,2}); % set the viewpoint

    if sim_config.show_titles
        % set the title of each viewpoint:
        ht = title(sim_config.vwpts_annot{1,idx}, 'Interpreter', 'none', ...
                   'FontSize', sim_config.tit_font_sz, 'FontWeight', 'normal', ...
                   'Color', sim_config.tit_font_color);
        % justify the title on the left bottom side ...
        ht.HorizontalAlignment = 'left';
        ht.Units               = 'normalized';
        ht.Position = horzcat(0.05, -0.08, ht.Position(1,3));
    end
end

function animateSimRobot(fkin_jnts, sim_config, sim_tstep, nSteps, vis_ctrl)
    % create and draw the initial shape of the robot in base position ...
    [sim_config, hSimRobot, fkin_jnts, nRGObj] = createSimRobot(sim_config, fkin_jnts, vis_ctrl);

    nJnts = sim_config.robot_body.nJnts;
    nLnks = sim_config.robot_body.nLnks;
    nFeet = sim_config.robot_body.nFeet;
    nAxes = sim_config.nAxes;

    jnt_pair_idx = sim_config.robot_body.jnt_pair_idx;

    vis_speed = vis_ctrl.vis_speed;
    drawSkel  = vis_ctrl.drawSkel;
    drawBody  = vis_ctrl.drawBody;

    t = 2; % (t = 1 is initial base position)

    if (drawSkel && drawBody)
        % animate the full body of the robot (skeleton and the link shapes):
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx1 = 3;
        idx2 = 2 + nLnks;
        idx3 = idx2 + 1;

        while (t <= nSteps)
            tic; % start visualization step timer (for adapting the visualization speed)
            % update the forward kinematic translations (positions) of the joint nodes ...
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            % update the position parameter matrix of the links of the robot ...
            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            % update the link lines of the skeleton with respect to the new joint positions ...
            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            % update the shape positions of the corresponding link shapes (hull) of the robot ...
            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);
            % update in the first subplot all graphics objects of the robot ...
            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;

            % delete all old graphic objects in the remaining subplots and copy the updated
            % graphic objects to a specified array of each subplot. The new graphics elements
            % are children of the current axes handle (subplots):
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            % update the visualization speed to keep the simulation very close to real time ...
            % (note: this part can't transfered into an own function, else the
            %        simulation will be very slow.)
            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff); % make short break ...
            else
                % increase the visualization speed by 1 ...
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    elseif drawSkel
        % animate only the skeleton of the robot:
        idx1 = 3;
        idx2 = 2 + nLnks;

        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    elseif drawBody
        % animate only the link shapes (hull) of the robot:
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx3 = 3 + nLnks;

        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    else
        % animate only the joint nodes (data points) of the robot:
        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    end
end

function vidfrms = makeSimRobotVideo(fkin_jnts, sim_config, nSteps, vis_ctrl)
    % allocate the data structure for the video frames ...
    vidfrms(1:nSteps,1) = struct('cdata', [], 'colormap', []);
    % create and draw the initial shape of the robot in base position ...
    [sim_config, hSimRobot, fkin_jnts, nRGObj, vidfrms(1,1)] = createSimRobot(sim_config, fkin_jnts, vis_ctrl);

    nJnts = sim_config.robot_body.nJnts;
    nLnks = sim_config.robot_body.nLnks;
    nFeet = sim_config.robot_body.nFeet;
    nAxes = sim_config.nAxes;

    jnt_pair_idx = sim_config.robot_body.jnt_pair_idx;

    drawSkel = vis_ctrl.drawSkel;
    drawBody = vis_ctrl.drawBody;

    if (sim_config.vid_axes_idx > 0)
        % capture only the specified subplot frame ...
        hObj = gca;
    else
        % capture the entire figure window ...
        hObj = sim_config.hWndFigure;
    end

    t = 2;

    if (drawSkel && drawBody)
        % animate the full body of the robot (skeleton and the link shapes):
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx1 = 3;
        idx2 = 2 + nLnks;
        idx3 = idx2 + 1;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            % capture the current state of the axes (frame) for the video ...
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    elseif drawSkel
        % animate only the skeleton of the robot:
        idx1 = 3;
        idx2 = 2 + nLnks;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    elseif drawBody
        % animate only the link shapes (hull) of the robot:
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx3 = 3 + nLnks;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    else
        % animate only the joint nodes (data points) of the robot:
        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    end
end

function animateSimRobotPL(fkin_jnts, vqT_vb, vb_idx, nVBds, sim_config, sim_tstep, nSteps, vis_ctrl)
    [sim_config, hSimRobot, fkin_jnts, nRGObj] = createSimRobot(sim_config, fkin_jnts, vis_ctrl);

    nJnts = sim_config.robot_body.nJnts;
    nLnks = sim_config.robot_body.nLnks;
    nFeet = sim_config.robot_body.nFeet;
    nAxes = sim_config.nAxes;

    jnt_pair_idx = sim_config.robot_body.jnt_pair_idx;

    vis_speed = vis_ctrl.vis_speed;
    drawSkel  = vis_ctrl.drawSkel;
    drawBody  = vis_ctrl.drawBody;

    t = 2;

    % animate the robot and the payload objects:
    if (drawSkel && drawBody)
        % full robot body (skeleton and the link shapes):
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx1 = 3;
        idx2 = 2 + nLnks;
        idx3 = idx2 + 1;

        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    elseif drawSkel
        % only the skeleton of the robot:
        idx1 = 3;
        idx2 = 2 + nLnks;

        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    elseif drawBody
        % only the link shapes (hull) of the robot:
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx3 = 3 + nLnks;

        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    else
        % only the joint nodes (data points) of the robot:
        while (t <= nSteps)
            tic;
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);

            tdiff = vis_speed * sim_tstep - toc();
            if (tdiff > 0)
                pause(tdiff);
            else
                vis_speed = vis_speed + 1;
            end
            t = t + vis_speed;
        end
    end
end

function vidfrms = makeSimRobotVideoPL(fkin_jnts, vqT_vb, vb_idx, nVBds, sim_config, nSteps, vis_ctrl)
    % allocate the data structure for the video frames ...
    vidfrms(1:nSteps,1) = struct('cdata', [], 'colormap', []);

    [sim_config, hSimRobot, fkin_jnts, nRGObj, vidfrms(1,1)] = createSimRobot(sim_config, fkin_jnts, vis_ctrl);

    nJnts = sim_config.robot_body.nJnts;
    nLnks = sim_config.robot_body.nLnks;
    nFeet = sim_config.robot_body.nFeet;
    nAxes = sim_config.nAxes;

    jnt_pair_idx = sim_config.robot_body.jnt_pair_idx;

    drawSkel = vis_ctrl.drawSkel;
    drawBody = vis_ctrl.drawBody;

    if (sim_config.vid_axes_idx > 0)
        % capture only the specified subplot frame ...
        hObj = gca;
    else
        % capture the entire figure window ...
        hObj = sim_config.hWndFigure;
    end

    t = 2;

    % animate the robot and the payload objects:
    if (drawSkel && drawBody)
        % full robot body (skeleton and the link shapes):
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx1 = 3;
        idx2 = 2 + nLnks;
        idx3 = idx2 + 1;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            % capture the current state of the axes (frame) for the video ...
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    elseif drawSkel
        % only the skeleton of the robot:
        idx1 = 3;
        idx2 = 2 + nLnks;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx1:idx2,1) = updateRobotSkeleton(hSimRobot(idx1:idx2,1), fkin_jnts.pair_pos, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    elseif drawBody
        % only the link shapes (hull) of the robot:
        shape_geom = sim_config.robot_body.shape_geom;
        foot_geom  = sim_config.robot_body.foot_geom;

        idx3 = 3 + nLnks;

        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            hSimRobot(idx3:nRGObj,1) = updateRobotBody(hSimRobot(idx3:nRGObj,1), fkin_jnts.pair_pos, ...
                                                       nLnks, nFeet, foot_geom, shape_geom.size_sf);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    else
        % only the joint nodes (data points) of the robot:
        while (t <= nSteps)
            [hSimRobot(1:nJnts,1), fkin_jnts] = updateJointNodes(hSimRobot(1:nJnts,1), fkin_jnts, nJnts, t);

            fkin_jnts.pair_pos = getLinkPositions(fkin_jnts.pos, jnt_pair_idx, nLnks);

            sim_config.gfx_objects{1,1}(1:nRGObj,1) = hSimRobot;
            sim_config.gfx_objects{1,1} = updatePayloadObj(sim_config.gfx_objects{1,1}, t, nRGObj, ...
                                                           sim_config.environment.vb_objects, vqT_vb, vb_idx, nVBds);
            sim_config.gfx_objects = updateSimAxes(sim_config.gfx_objects, sim_config.hWndFigure, ...
                                                   sim_config.hAxes, nAxes);
            vidfrms(t,1) = getframe(hObj);
            t = t + 1;
        end
    end
end

function gobj_arr = updatePayloadObj(gobj_arr, t, nRGObj, vb_objects, vqT_vb, vb_idx, nVBds)
    i = -6;
    for j = 1:nVBds
        i  = i + 7;
        io = vb_idx(j,1); % object index (volume body)
        ig = nRGObj + io; % graphic object index

        vb_obj = vb_objects(io,1);
        vb_obj.frame = vqT_vb(i:7*j,t);

        gobj_arr(ig,1) = vb_obj.updGObj(gobj_arr(ig,1));
    end
end

function gfx_objects = updateSimAxes(gfx_objects, hWndFigure, hAxes, nAxes)
    if (nAxes > 1)
        for i = 2:nAxes
            set(hWndFigure, 'CurrentAxes', hAxes(1,i));
            cla; % delete all graphics objects from the current axes ...

            delete(gfx_objects{1,i}); % remove all graphics objects from the memory ...
            gfx_objects{1,i} = copyobj(gfx_objects{1,1}, hAxes(1,i));
        end
    end
    drawnow;
end

function [hJntNodes, fkin_jnts] = createJointNodes(fkin_jnts, sim_config, vis_ctrl)
    nJnts = sim_config.robot_body.nJnts;

    % plot properties for the joint nodes of the robot's skeleton ...
    jnt_plot_prop = sim_config.robot_body.draw_prop.joints;
    if ~vis_ctrl.drawJnts
        jnt_plot_prop.marker = 'none';
        jnt_plot_prop.color  = 'none';
    end
    % plot properties for the center of mass (CoM) ...
    com_plot_prop = sim_config.robot_body.draw_prop.com;
    if ~vis_ctrl.drawCom
        com_plot_prop.marker = 'none';
        com_plot_prop.color  = 'none';
    end

    % draw the the joints and the CoM of the robot's skeleton:
    fk_vqT = squeeze(fkin_jnts.vqT(1,1:7,1:nJnts)).';
    [hJntNodes, fkin_jnts.pos(1:nJnts,1:3)] = plotFKinJointPositions(fkin_jnts.pos, fk_vqT, nJnts, jnt_plot_prop, com_plot_prop);
end

function [vhdp_j, vpos_j] = plotFKinJointPositions(vpos_j, vqT_j, nJnts, jnt_plot_prop, com_plot_prop)
    k = nJnts - 1;
    vpos_j(1:nJnts,1:3) = vqT_j(1:nJnts,1:3);
    vhdp_j = gobjects(2,1); % initialize array for graphic objects (data points)

    % draw the data points (nodes) of the joints of the robot's skeleton:
    vhdp_j(1,1) = plot3(vpos_j(1,1), vpos_j(1,2), vpos_j(1,3), 'LineStyle', 'none', 'Marker', jnt_plot_prop.marker, ...
                        'MarkerSize', jnt_plot_prop.marker_sz, 'MarkerEdgeColor', jnt_plot_prop.color);
    set(vhdp_j(1,1), 'XData', vpos_j(2:k,1), 'YData', vpos_j(2:k,2), 'ZData', vpos_j(2:k,3));

    % draw the position of the center of mass (CoM):
    vhdp_j(2,1) = plot3(vpos_j(nJnts,1), vpos_j(nJnts,2), vpos_j(nJnts,3), 'LineStyle', 'none', 'Marker', com_plot_prop.marker, ...
                        'MarkerSize', com_plot_prop.marker_sz, 'MarkerEdgeColor', com_plot_prop.color);
end

function [vhdp_j, vpos_j] = updateFKinJointPositions(vhdp_j, vpos_j, vqT_j, nJnts)
    k = nJnts - 1;
    vpos_j(1:nJnts,1:3) = vqT_j(1:nJnts,1:3); % get the new translations ...

    % update the positions of all 3D data-points ...
    set(vhdp_j(1,1), 'XData', vpos_j(1:k,1), 'YData', vpos_j(1:k,2), 'ZData', vpos_j(1:k,3));       % joint nodes
    set(vhdp_j(2,1), 'XData', vpos_j(nJnts,1), 'YData', vpos_j(nJnts,2), 'ZData', vpos_j(nJnts,3)); % CoM
end

function jnt_pair_pos = getLinkPositions(vpos_j, jnt_pair_idx, nLnks)
    % compute the position parameter matrix for the links (joint pairs)
    % of the robot with a predefined joint pair list:
    jnt_pair_pos = zeros(nLnks,6);
    for i = 1:nLnks
        idx = jnt_pair_idx(i,1:6);
        %                                          x1                  x2                  y1
        jnt_pair_pos(i,1:6) = horzcat( vpos_j(idx(1,1),1), vpos_j(idx(1,2),1), vpos_j(idx(1,3),2), ...
                                       vpos_j(idx(1,4),2), vpos_j(idx(1,5),3), vpos_j(idx(1,6),3) );
        %                                          y2                  z1                  z2
    end
end

function shape_vtx = getLinkShapeVertices(jnt_pair_pos, size_sf)
    % compute the hull vertices around the current link of the robot:
    % determine the orientation of the current link ...
    %                                x2                  x1
    %                                y2                  y1
    %                                z2                  z1
    lnk_vec = horzcat( jnt_pair_pos(1,2) - jnt_pair_pos(1,1), ...
                       jnt_pair_pos(1,4) - jnt_pair_pos(1,3), ...
                       jnt_pair_pos(1,6) - jnt_pair_pos(1,5) );

    % calculate the orthonormal basis for the null space (kernel) of "lnk_vec" ...
    onb_lnk = null(lnk_vec); % orthonormal vectors to the link
    % scale the orthonormal vectors ...
    on_sc_vec_1 = size_sf(1,1) * onb_lnk(:,1);
    on_sc_vec_2 = size_sf(1,2) * onb_lnk(:,2);

    % calculate the offsets in the direction orthogonal to the link ...
    ofs = zeros(4,3);
    ofs(1,1:3) =  on_sc_vec_1 + on_sc_vec_2;
    ofs(2,1:3) = -on_sc_vec_1 + on_sc_vec_2;
    ofs(3,1:3) = -on_sc_vec_1 - on_sc_vec_2;
    ofs(4,1:3) =  on_sc_vec_1 - on_sc_vec_2;

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

function shape_lnk = createLinkShape(jnt_pair_pos, size_sf, faces, draw_prop)
    % compute a simple hull (box shape) around the current link of the robot:
    shape_vtx = getLinkShapeVertices(jnt_pair_pos, size_sf);
    % create several filled polygons to form the hull (box shape) for the current link ...
    shape_lnk = patch('Vertices', shape_vtx, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                      'Faces', faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function shape_lnk = updateLinkShapePos(shape_lnk, jnt_pair_pos, size_sf)
    % compute the new vertex positions for the box shape of the current link ...
    new_vtx_pos = getLinkShapeVertices(jnt_pair_pos, size_sf);
    % update the position of the shape ...
    shape_lnk.Vertices = new_vtx_pos;
end

function vtcs_ft = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds)
    % scaled orthogonal (lin. independent) vectors to the current foot joint ...
    o_vec_1 = vertcat(0, base_sz.width, 0);
    o_vec_2 = vertcat(0, 0, base_sz.height);

    % calculate the offsets in the direction orthogonal to the foot joint ...
    ofs = zeros(4,3);
    ofs(1,1:3) =  o_vec_1 + 2*o_vec_2;
    ofs(2,1:3) = -o_vec_1 + 2*o_vec_2;
    ofs(3,1:3) = -o_vec_1 - o_vec_2;
    ofs(4,1:3) =  o_vec_1 - o_vec_2;

    % compute the vertices of each patch to form the foot shape ...
    idx = 1;
    vtcs_ft = zeros(8,3);
    for i = 1:2 % while (idx <= 8)
        for j = 1:4
            vtcs_ft(idx,1:3) = horzcat( jnt_pair_pos(1,2) + ofs(j,1) + foot_ds(idx,1), ...
                                        jnt_pair_pos(1,4) + ofs(j,2) + foot_ds(idx,2), ...
                                        jnt_pair_pos(1,6) + ofs(j,3) + foot_ds(idx,3) );
            idx = idx + 1;
        end
    end
end

function shape_ft = createFootShape(jnt_pair_pos, base_sz, foot_ds, faces, draw_prop)
    % compute the vertex positions to build the shape of the current foot of the robot ...
    vtcs_ft = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds);
    % create several filled polygons to form the shape of the foot ...
    shape_ft = patch('Vertices', vtcs_ft, 'LineWidth', draw_prop.line_width, 'EdgeColor', draw_prop.edge_color, ...
                     'Faces', faces, 'FaceColor', draw_prop.face_color, 'FaceAlpha', draw_prop.face_alpha);
end

function shape_ft = updateFootPos(shape_ft, jnt_pair_pos, base_sz, foot_ds)
    % compute the new vertex positions for the shape of the foot ...
    new_vtx_pos = getFootShapeVertices(jnt_pair_pos, base_sz, foot_ds);
    % update the position of the foot ...
    shape_ft.Vertices = new_vtx_pos;
end

function hLnkLines = createRobotSkeleton(jnt_pair_pos, nLnks, draw_prop)
    hLnkLines = gobjects(nLnks,1); % initialize the graphic objects array (link edges)

    % draw the lines (edges) that are depicting the links to form
    % the kinematic chain, the skeleton, of the robot:
    for i = 1:nLnks
        hLnkLines(i,1) = animatedline('LineWidth', draw_prop.line_width, 'Color', draw_prop.color);
        %                             transl. (x1,x2),     transl. (y1,y2),     transl. (z1,z2)
        addpoints(hLnkLines(i,1), jnt_pair_pos(i,1:2), jnt_pair_pos(i,3:4), jnt_pair_pos(i,5:6));
    end
end

function hLnkShapes = createRobotBody(jnt_pair_pos, nLnks, nFeet, foot_geom, shape_geom, draw_prop)
    nShpElms   = nLnks + nFeet;
    hLnkShapes = gobjects(nShpElms,1); % initialize array for graphic objects (link shapes)

    % draw the hull (box shapes) for the corresponding links of the robot:
    for i = 1:nLnks
        % create the shape of the link, which is around the link line ...
        hLnkShapes(i,1) = createLinkShape(jnt_pair_pos(i,1:6), shape_geom.size_sf(i,1:2), shape_geom.faces, draw_prop);
    end

    % draw the foot shapes ...
    for i = 1:nFeet
        jnt_idx = foot_geom.joints(i);
        hLnkShapes(nLnks+i,1) = createFootShape(jnt_pair_pos(jnt_idx,1:6), foot_geom.base_sz, foot_geom.shape_ds, ...
                                                shape_geom.faces, draw_prop);
    end
end

function [hJntNodes, fkin_jnts] = updateJointNodes(hJntNodes, fkin_jnts, nJnts, t)
    new_fk_vqT = squeeze(fkin_jnts.vqT(t,1:7,1:nJnts)).';
    [hJntNodes, fkin_jnts.pos] = updateFKinJointPositions(hJntNodes, fkin_jnts.pos, new_fk_vqT, nJnts);
end

function hLnkLines = updateRobotSkeleton(hLnkLines, new_jnt_pair_pos, nLnks)
    % update the links (edge positions) of the robot's skeleton:
    for i = 1:nLnks
        clearpoints(hLnkLines(i,1));
        %                                new pos. (x1,x2),        new pos. (y1,y2),        new pos. (z1,z2)
        addpoints(hLnkLines(i,1), new_jnt_pair_pos(i,1:2), new_jnt_pair_pos(i,3:4), new_jnt_pair_pos(i,5:6));
    end
end

function hLnkShapes = updateRobotBody(hLnkShapes, new_jnt_pair_pos, nLnks, nFeet, foot_geom, size_sf)
    % update the shape positions of the corresponding links of the robot:
    for i = 1:nLnks
        hLnkShapes(i,1) = updateLinkShapePos(hLnkShapes(i,1), new_jnt_pair_pos(i,1:6), size_sf(i,1:2));
    end

    % update the feet positions ...
    for i = 1:nFeet
        jnt_idx = foot_geom.joints(i);
        hLnkShapes(nLnks+i,1) = updateFootPos(hLnkShapes(nLnks+i,1), new_jnt_pair_pos(jnt_idx,1:6), ...
                                              foot_geom.base_sz, foot_geom.shape_ds);
    end
end
