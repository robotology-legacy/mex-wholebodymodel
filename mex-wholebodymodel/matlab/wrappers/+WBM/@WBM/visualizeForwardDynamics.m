function visualizeForwardDynamics(obj, pos_out, sim_config, sim_tstep, vis_ctrl)
    switch nargin
        case 4
            % draw all graphic elements of the simulated robot ...
            vis_ctrl.drawJnts = true;
            vis_ctrl.drawCom  = true;
            vis_ctrl.drawSkel = true;
            vis_ctrl.drawBody = true;
            % use the default visualization speed value ...
            % Note: Use a different speed value in order to make the visualization speed
            %       close to real-time when the simulation time step is changed.
            vis_ctrl.vis_speed = 1.0;
        case 5
            if ~isstruct(vis_ctrl)
                error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ~isfield(vis_ctrl, 'vis_speed')
                % add default speed value ...
                vis_ctrl.vis_speed = 1.0;
            end
        otherwise
            error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % initial plot of the animation environment (reset all subplots in the
    % main figure and reinitialize them with a new ground floor) ...
    sim_config = initSimEnvironment(sim_config);

    visualizeSimRobot(obj, pos_out, sim_config, sim_tstep, vis_ctrl);
end
%% END of visualizeForwardDynamics.


%% ENVIRONMENT INITIALIZATION & LIGHT:

function sim_config = initSimEnvironment(sim_config)
    nAxes = sim_config.nAxes;

    % clear and reset all subplots of the figure window (in reverse order):
    for i = nAxes:-1:1
        % set the axes of the subplot to the current axes ...
        set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,i));

        sim_config.gfx_objects{1,i} = []; % clear the graphics container
        cla; % delete all graphics objects from the current axes ...
        axis off;
        hold on; % retain new plots in the current axes ...
    end

    % draw the colored ground (rectangle) and the corresponding origin point of the axis:
    % note: for correct redrawing of the scene, the ground objects will be stored in reverse order.
    gfx_ground = gobjects(2,1);
    gfx_ground(2,1) = fill3(sim_config.environment.grnd_shape(1,1:4), sim_config.environment.grnd_shape(2,1:4), ...
                            sim_config.environment.grnd_shape(3,1:4), sim_config.environment.grnd_color, ...
                            'EdgeColor', sim_config.environment.grnd_edge_color);
    gfx_ground(1,1) = plot3(0, 0, 0, 'Marker', '.', 'MarkerSize', sim_config.environment.orig_pt_size, ...
                            'MarkerEdgeColor', sim_config.environment.orig_pt_color);
    sim_config.gfx_objects{1,1} = gfx_ground;

    % if activated, set and show a point light:
    if sim_config.show_light
        gldta = opengl('data');
        hlt = setPointLight(sim_config.light_pos, gldta.Software);
        sim_config.gfx_objects{1,1} = vertcat(sim_config.gfx_objects{1,1}, hlt);
    end

    % if some volume bodies for the scenario are given, create and draw these
    % volume bodies in the simulation environment:
    vb_objects = sim_config.environment.vb_objects;
    if ~isempty(vb_objects)
        if ~iscolumn(vb_objects)
            error('initSimEnvironment: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
        end
        nObj    = size(vb_objects,1);
        gfx_vbo = gobjects(nObj,1);

        for i = 1:nObj
            if (vb_objects(i,1).frame ~= vb_objects(i,1).init_frame)
                % set the frame to the initial condition ...
                vb_objects(i,1).setInitFrame();
            end
            gfx_vbo(i,1) = vb_objects(i,1).getGObj();
        end
        % add the volume body objects to the graphics list ...
        sim_config.gfx_objects{1,1} = vertcat(gfx_vbo, sim_config.gfx_objects{1,1});
    end
end

function hlt = setPointLight(lpos, srender)
    % set the light properties:
    hlt = light(gca, 'Position', lpos, 'Style', 'local', 'Visible', 'on');
    if ~srender
        % hardware-accelerated rendering ...
        lighting gouraud;
    else
        % software rendering ...
        lighting flat;
    end
    material dull;
end
