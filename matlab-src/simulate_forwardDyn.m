exp_count =0;
for ind_exp1=1:ind_exp1max
 
    if EXPERIMENT_MODE ==1; eval(strcat('params.',var_exp1,'=array_var_exp1(',num2str(ind_exp1),');'));end;
    
    for ind_exp2=1:ind_exp2max
    
        if EXPERIMENT_MODE ==1
            eval(strcat('params.',var_exp2,'=array_var_exp2(',num2str(ind_exp2),');'));
            update_parameters;
        end
    
        
        exp_count=exp_count+1;
        
        
        params.results.contactForces = [];
        params.results.jointTorques = [];
        params.results.comTraj=[];
        
        
        set(gui_edit_messages,'String',[{'...'};{'...'};get(gui_edit_messages,'String') ])
        set(gui_edit_messages,'String',[{strcat('Duration: ',num2str(params.sim_duration,3),' sec')};get(gui_edit_messages,'String') ])
        set(gui_edit_messages,'String',[{strcat('Time step: ',num2str(params.sim_step,3),' sec')};get(gui_edit_messages,'String') ])
        %refresh tspan for interpolation and visualizer
        params.tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

        % disp('Starting Integration');
        tic; % integration start
        set(gui_edit_messages,'String',[{'Starting integration...'};get(gui_edit_messages,'String') ])
        drawnow;
        
        options = odeset('RelTol',1e-3,'AbsTol',1e-6,'Minimum');
        
        [t,xout] = ode15s(@(t,qv)func_forwardDyn(t, qv, params),[params.sim_start_time params.sim_start_time+params.sim_duration],params.qvInit,options);
        
        %interpolation of the output
        xout = transpose(spline(t',xout',params.tSpan));
        t    = transpose(params.tSpan);
        set(gui_edit_messages,'String',[{strcat('Integration took: ',num2str(toc(),3),' sec')};get(gui_edit_messages,'String') ])
        drawnow;
        
        %computation of the requested results
        if params.PLOT.contactForces == 1;params.results.contactForces = zeros(size(params.tSpan,2),12);end;
        if params.PLOT.jointTorques  == 1;params.results.jointTorques = zeros(size(params.tSpan,2),params.n_dof);end;
        if params.PLOT.comTraj       == 1;params.results.comTraj = zeros(size(params.tSpan,2),9);end;
        
        for ii=1:size(params.tSpan,2)
            [ qvDot, F_contact,tau_ctrl, comTraj] = func_forwardDyn(t(ii), xout(ii,:)', params );
            
            if params.PLOT.contactForces == 1;params.results.contactForces(ii,:) = F_contact';end;
            if params.PLOT.jointTorques  == 1;params.results.jointTorques(ii,:) = tau_ctrl';end;
            if params.PLOT.comTraj       == 1;params.results.comTraj(ii,:) = comTraj;end;
        end
        
        %presenting the requested results
        if params.PLOT.contactForces == 1
            if exist('figure_contactForces','var')
                close(figure_contactForces);
                clear figure_contactForces;
            end
            figure_contactForces = figure('Name', 'iCub Simulator - Contact Forces', 'NumberTitle', 'off',...
                'Position', [675,550,1250,450]);
            subplot(2,2,1);
            plot(t,params.results.contactForces(:,1:3));
            title('Left Foot');xlabel('Time (s)');ylabel('Fc');legend('F_x','F_y','F_z');
            
            subplot(2,2,2);
            plot(t,params.results.contactForces(:,7:9));
            title('Right Foot');xlabel('Time (s)');ylabel('Fc');legend('F_x','F_y','F_z');
            
            subplot(2,2,3);
            plot(t,params.results.contactForces(:,4:6));
            title('Left Foot');xlabel('Time (s)');ylabel('Mc');legend('M_x','M_y','M_z');
            
            subplot(2,2,4);
            plot(t,params.results.contactForces(:,10:12));
            title('Right Foot');xlabel('Time (s)');ylabel('Mc');legend('M_x','M_y','M_z');
            
        end
        
        if params.PLOT.jointTorques  == 1
            if exist('figure_jointTorques','var')
                close(figure_jointTorques);
                clear figure_jointTorques;
            end
            
            figure_jointTorques = figure('Name', 'iCub Simulator - Joint Torques', 'NumberTitle', 'off',...
                'Position', [50,0,600,300]);
            plot_torques = subplot('Position',[0.075,0.15,0.75,0.8]);
            
            gui_pushbutton_joints_torso = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 260 80 17],...
                'String','Torso','Callback','plot(t,params.results.jointTorques(:,1:3));legend toggle;title(''Joint Torques - Torso'');xlabel(''Time (s)'');ylabel(''tau'');');
            gui_pushbutton_joints_l_arm = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 240 80 17],...
                'String','l_arm','Callback','plot(t,params.results.jointTorques(:,4:8));legend toggle;title(''Joint Torques - Left Arm'');xlabel(''Time (s)'');ylabel(''tau'');');
            gui_pushbutton_joints_r_arm = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 220 80 17],...
                'String','r_arm','Callback','plot(t,params.results.jointTorques(:,9:13));legend toggle;title(''Joint Torques - Right Arm'');xlabel(''Time (s)'');ylabel(''tau'');');
            gui_pushbutton_joints_l_leg = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 200 80 17],...
                'String','l_leg','Callback','plot(t,params.results.jointTorques(:,14:19));legend toggle;title(''Joint Torques - Left Leg'');xlabel(''Time (s)'');ylabel(''tau'');');
            gui_pushbutton_joints_r_leg = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 180 80 17],...
                'String','r_leg','Callback','plot(t,params.results.jointTorques(:,20:25));legend toggle;title(''Joint Torques - Right Leg'');xlabel(''Time (s)'');ylabel(''tau'');');
            
            gui_pushbutton_joints_all = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 160 80 17],...
                'String','ALL','Callback','plot(t,params.results.jointTorques);');
            
            
            plot(t,params.results.jointTorques);
            title('Joint Torques');xlabel('Time (s)');ylabel('tau');
            
        end
        
        if params.PLOT.comTraj == 1
            if exist('figure_comTraj','var')
                close(figure_comTraj);
                clear figure_comTraj;
            end
            figure_comTraj = figure('Name', 'iCub Simulator - Center of Mass Trajectories', 'NumberTitle', 'off',...
                'Position', [675,0,1250,450]);
            subplot(1,3,1);
            plot(t,params.results.comTraj(:,1:3));
            title('x_{com} error');xlabel('Time (s)');ylabel('Position error');legend('e_x','e_y','e_z');
            
            subplot(1,3,2);
            plot(t,params.results.comTraj(:,4:6)-params.results.comTraj(:,7:9));
            title('ddx_{com} error');xlabel('Time (s)');ylabel('Acceleration error');legend('x','y','z');
            
            subplot(1,3,3);
            plot(t,params.results.comTraj(:,7:9));
            title('ddx_{com} desired');xlabel('Time (s)');ylabel('Acc');legend('x','y','z');
            
            
        end
        %
        
        
        
        
%         xout = xout(:,1:end-3); %IntCoMerror reduced
        figure(figure_main);
        
        if VISUALIZER_ON == 1;
%             exp_count
%             params.sim_duration
%             params.sim_step
            
            visualize_forwardDyn(xout,params);
            
            set(gui_edit_messages,'String',[{'Visualization finished'};get(gui_edit_messages,'String') ])
            
        end
        
        if EXPERIMENT_MODE==1
            eval(strcat('params.results.exp',num2str(exp_count),'.comTraj = params.results.comTraj;'));
            eval(strcat('params.results.exp',num2str(exp_count),'.contactForces = params.results.contactForces;'));
            eval(strcat('params.results.exp',num2str(exp_count),'.jointTorques = params.results.jointTorques;'));
        else
            params.matfile.comTraj = params.results.comTraj;
            params.matfile.contactForces = params.results.contactForces;
            params.matfile.jointTorques = params.results.jointTorques;
        end
        
    end
end

if EXPERIMENT_MODE == 1
    results_data = params.results; 
    save results.mat results_data;
    movefile('results.mat',strcat('./experiment_results/results',datestr(now,30),'.mat'))
end