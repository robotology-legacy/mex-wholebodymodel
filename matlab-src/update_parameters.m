
params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

params.qjInit = zeros(size(params.qjInit));
tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

params.controller.Impedances  = [params.kImpTorso,params.kImpArms,params.kImpArms,params.kImpLegs,params.kImpLegs];

params.controller.qjInit           = params.qjInit;    

wholeBodyModel('update-state',params.qjInit,zeros(params.n_dof,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

params.qvInit = [T_baseInit;params.qjInit;params.qjDotInit;params.v_baseInit];


%params.qvInit = [wholeBodyModel('forward-kinematics','l_sole');params.qjInit;params.qjDotInit;params.v_baseInit];
%params.qvInit = [wholeBodyModel('forward-kinematics','root_link');params.qjInit;params.qjDotInit;params.v_baseInit];
%params.qvInit = [params.qvInit;0;0;0]; %intComError added 

if EXPERIMENT_MODE==0
    ind_exp1max=1;ind_exp2max=1;
else
    ind_exp1max = size(array_var_exp1,1);
    ind_exp2max = size(array_var_exp2,1);    
end

%%
disp('Parameters updated.');
