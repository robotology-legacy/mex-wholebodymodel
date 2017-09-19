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
    initSimEnvironment(sim_config);

    visualizeSimRobot(obj, pos_out, sim_config, sim_tstep, vis_ctrl);
end
%% END of visualizeForwardDynamics.


%% ENVIRONMENT INITIALIZATION:

function initSimEnvironment(sim_config)
    % clear and reset all subplots of the figure window in the reverse order ...
    for i = sim_config.nAxes:-1:1
        % set the axes of the current subplot in the figure ...
        set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,i));
        cla; % delete all graphics objects from the current axes
        axis off;
        % redraw the ground (rectangle) with a new color ...
        fill3(sim_config.environment.grnd_shape(1,1:4), sim_config.environment.grnd_shape(2,1:4), ...
              sim_config.environment.grnd_shape(3,1:4), sim_config.environment.grnd_color, ...
              'EdgeColor', sim_config.environment.grnd_edge_color);
        % draw the origin point of the axis onto the ground ...
        plot3(0, 0, 0, 'Marker', '.', 'MarkerSize', sim_config.environment.orig_pt_size, ...
              'MarkerEdgeColor', sim_config.environment.orig_pt_color);
        drawnow; % update and draw the current subplot in the figure window
    end
end
