
params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

params.tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

params.controller.Impedances  = [params.kImpTorso,params.kImpArms,params.kImpArms,params.kImpLegs,params.kImpLegs];

params.controller.qjInit           = params.qjInit;    

wholeBodyModel('update-state',params.qjInit,zeros(params.n_dof,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

params.qvInit = [T_baseInit;params.qjInit;params.qjDotInit;params.v_baseInit];

params.qvInit = [params.qvInit;0;0;0]; %intComError added 

%%
disp('Parameters updated.');
