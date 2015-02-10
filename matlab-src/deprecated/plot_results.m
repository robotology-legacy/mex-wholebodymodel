function plot_results(varargin)

if nargin==2
    exp_name = varargin{2};
else
    exp_name = 'Sim';
end    
t = varargin{1}.t;
xout = varargin{1}.xout;
params = varargin{1}.params;
n_constraint = params.n_constraint;

tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

%computation of the requested results
%         if params.PLOT.contactForces == 1;contactForces = zeros(size(tSpan,2),12);end;
contactForces = zeros(size(tSpan,2),6*n_constraint);
%         if params.PLOT.jointTorques  == 1;jointTorques = zeros(size(tSpan,2),params.n_dof);end;
jointTorques = zeros(size(tSpan,2),params.n_dof);
%         if params.PLOT.comTraj       == 1;comTraj = zeros(size(tSpan,2),9);end;
comTraj = zeros(size(tSpan,2),6+n_constraint*6);

for ii=1:size(tSpan,2)
    [ qvDot, F_contact,tau_ctrl, CoM] = func_forwardDyn(t(ii), xout(ii,:)', params );
    
    %     if params.PLOT.contactForces == 1;contactForces(ii,:) = F_contact';end;
    contactForces(ii,:) = F_contact';
    %     if params.PLOT.jointTorques  == 1;jointTorques(ii,:) = tau_ctrl';end;
    jointTorques(ii,:) = tau_ctrl';
    %     if params.PLOT.comTraj       == 1;comTraj(ii,:) = CoM;end;
    comTraj(ii,:) = CoM;
    
end

%presenting the requested results
%         if params.PLOT.contactForces == 1
%             if exist('figure_contactForces','var')
%                 close(figure_contactForces);
%                 clear figure_contactForces;
%             end
figure_contactForces = figure('Name', strcat('iCub Simulator - Contact Forces - exp',exp_name), 'NumberTitle', 'off',...
    'Position', [675,550,1250,450]);
switch n_constraint
    case 1 % on left foot
        subplot(1,2,1);
        plot(t,contactForces(:,1:3));
        title('Left Foot - Forces');xlabel('Time (s)');ylabel('Fc');legend('F_x','F_y','F_z');
        
        subplot(1,2,2);
        plot(t,contactForces(:,4:6));
        title('Left Foot - Moments');xlabel('Time (s)');ylabel('Mc');legend('M_x','M_y','M_z');
    case 2 % on both feet
        subplot(2,2,1);
        plot(t,contactForces(:,1:3));
        title('Left Foot - Forces');xlabel('Time (s)');ylabel('Fc');legend('F_x','F_y','F_z');
        
        subplot(2,2,3);
        plot(t,contactForces(:,4:6));
        title('Left Foot - Moments');xlabel('Time (s)');ylabel('Mc');legend('M_x','M_y','M_z');
        
        subplot(2,2,2);
        plot(t,contactForces(:,7:9));
        title('Right Foot - Forces');xlabel('Time (s)');ylabel('Fc');legend('F_x','F_y','F_z');
        
        subplot(2,2,4);
        plot(t,contactForces(:,10:12));
        title('Right Foot - Moments');xlabel('Time (s)');ylabel('Mc');legend('M_x','M_y','M_z');
    otherwise
        disp('Choose number of constraints properly (1 or 2)');
        return
end
%         end

%         if params.PLOT.jointTorques  == 1
%             if exist('figure_jointTorques','var')
%                 close(figure_jointTorques);
%                 clear figure_jointTorques;
%             end

figure_jointTorques = figure('Name', strcat('iCub Simulator - Joint Torques - exp',exp_name), 'NumberTitle', 'off',...
    'Position', [50,0,600,300]);
plot_torques = subplot('Position',[0.075,0.15,0.75,0.8]);

% gui_pushbutton_joints_torso = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 260 80 17],...
%     'String','Torso','Callback','plot(t,jointTorques(:,1:3));legend toggle;title(''Joint Torques - Torso'');xlabel(''Time (s)'');ylabel(''tau'');');
% gui_pushbutton_joints_l_arm = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 240 80 17],...
%     'String','l_arm','Callback','plot(t,jointTorques(:,4:8));legend toggle;title(''Joint Torques - Left Arm'');xlabel(''Time (s)'');ylabel(''tau'');');
% gui_pushbutton_joints_r_arm = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 220 80 17],...
%     'String','r_arm','Callback','plot(t,jointTorques(:,9:13));legend toggle;title(''Joint Torques - Right Arm'');xlabel(''Time (s)'');ylabel(''tau'');');
% gui_pushbutton_joints_l_leg = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 200 80 17],...
%     'String','l_leg','Callback','plot(t,jointTorques(:,14:19));legend toggle;title(''Joint Torques - Left Leg'');xlabel(''Time (s)'');ylabel(''tau'');');
% gui_pushbutton_joints_r_leg = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 180 80 17],...
%     'String','r_leg','Callback','plot(t,jointTorques(:,20:25));legend toggle;title(''Joint Torques - Right Leg'');xlabel(''Time (s)'');ylabel(''tau'');');
% 
% gui_pushbutton_joints_all = uicontrol(figure_jointTorques,'Style','pushbutton','Pos',[502 160 80 17],...
%     'String','ALL','Callback','plot(t,jointTorques);');


plot(t,jointTorques);
title('Joint Torques');xlabel('Time (s)');ylabel('tau');

%         end

%         if params.PLOT.comTraj == 1
%             if exist('figure_comTraj','var')
%                 close(figure_comTraj);
%                 clear figure_comTraj;
%             end
figure_comTraj = figure('Name', strcat('iCub Simulator - Center of Mass Trajectories and Constraint Check- exp',exp_name), 'NumberTitle', 'off',...
    'Position', [675,0,1250,450]);
subplot(1,3,1);
plot(t,comTraj(:,1:3));
title('x_{com} error');xlabel('Time (s)');ylabel('Position error');legend('e_x','e_y','e_z');

subplot(1,3,2);
plot(t,comTraj(:,4:6));
title('ddx_{com} error');xlabel('Time (s)');ylabel('Acceleration error');legend('x','y','z');

subplot(1,3,3);
plot(t,comTraj(:,7:6+n_constraint*2));
title('constraint Jddq + dJdq==0 check');xlabel('Time (s)');ylabel('0');


%         end