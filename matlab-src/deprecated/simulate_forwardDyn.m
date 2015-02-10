
exp_count =0;
for ind_exp1=1:ind_exp1max
 
    if EXPERIMENT_MODE ==1; eval(strcat('params.',var_exp1,'=array_var_exp1{',num2str(ind_exp1),'};'));end;
    
    for ind_exp2=1:ind_exp2max
    
        if EXPERIMENT_MODE ==1
            eval(strcat('params.',var_exp2,'=array_var_exp2{',num2str(ind_exp2),'};'));
            disp(' ')
            disp(' ')
            
            exp_count=exp_count+1
            disp(var_exp1)
            disp(eval(strcat('params.',var_exp1)))
            disp(var_exp2)
            disp(eval(strcat('params.',var_exp2)))
            disp(' ')
            
            update_parameters;
            
            disp(' ')
            disp(' ')
        end
    
        
        
        
        contactForces = [];
        jointTorques = [];
        comTraj=[];
        
        
        set(gui_edit_messages,'String',[{'...'};{'...'};get(gui_edit_messages,'String') ])
        set(gui_edit_messages,'String',[{strcat('Duration: ',num2str(params.sim_duration,3),' sec')};get(gui_edit_messages,'String') ])
        set(gui_edit_messages,'String',[{strcat('Time step: ',num2str(params.sim_step,3),' sec')};get(gui_edit_messages,'String') ])
        %refresh tspan for interpolation and visualizer
        tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

        % disp('Starting Integration');
        tic; % integration start
        set(gui_edit_messages,'String',[{'Starting integration...'};get(gui_edit_messages,'String') ])
        drawnow;
        
        options = odeset('RelTol',1e-3,'AbsTol',1e-6);%,'MaxStep',1e-2,'InitialStep',1e-8);
        [t,xout] = ode15s(@(t,qv)func_forwardDyn(t, qv, params),[params.sim_start_time params.sim_start_time+params.sim_duration],params.qvInit,options);
        
        %interpolation of the output
        xout = transpose(spline(t',xout',tSpan));
        t    = transpose(tSpan);
        set(gui_edit_messages,'String',[{strcat('Integration took: ',num2str(toc(),3),' sec')};get(gui_edit_messages,'String') ])
        drawnow;
        
        
%         xout = xout(:,1:end-3); %IntCoMerror reduced
        
        temp_results.xout = xout;
        temp_results.t = t;
        temp_results.params = params;
        
        if params.PLOT.all ==1
            plot_results(temp_results,num2str(exp_count));
        end

        figure(figure_main);
        
        if VISUALIZER_ON == 1;
%             exp_count
%             params.sim_duration
%             params.sim_step

            visualize_forwardDyn(xout,params);
            while visualize_loop == 1
                visualize_forwardDyn(xout,params);
            end
            
            set(gui_edit_messages,'String',[{'Visualization finished'};get(gui_edit_messages,'String') ])
            
        end
        
            
        if EXPERIMENT_MODE==1
%             eval(strcat('results.exp',num2str(exp_count),'.comTraj = comTraj;'));
%             eval(strcat('results.exp',num2str(exp_count),'.contactForces = contactForces;'));
%             eval(strcat('results.exp',num2str(exp_count),'.jointTorques = jointTorques;'));            
            
            eval(strcat('results.exp',num2str(exp_count),'= temp_results;'));
            eval(strcat('results.exp',num2str(exp_count),'.',var_exp1,'= array_var_exp1{',num2str(ind_exp1),'};'));
            eval(strcat('results.exp',num2str(exp_count),'.',var_exp2,'= array_var_exp2{',num2str(ind_exp2),'};'));
                        
%             eval(strcat('results.exp',num2str(exp_count),'.params = params;'));
%             eval(strcat('results.exp',num2str(exp_count),'.t = t;'));
        else
            results_matfile = temp_results;
        end
        
    end
end

if EXPERIMENT_MODE == 1
    results_data = results; 
    save results.mat results_data;
    movefile('results.mat',strcat('./experiment_results/results',datestr(now,30),'.mat'))
end