clearvars;
close all;

%% initialise mexWholeBodyModel
wbm_modelInitialize('icubGazeboSim');

%% setup params
params.ndof = 25;
param.dampingCoeff = 0.5;

%% initial conditions
% this is assuming a 25DoF iCub

initCond = 2; % 1 - standing pose, 2 - random pose
if( initCond == 1)
    %for standing pose
    params.torsoInit    = [-10.0  0.0   0.0]';%[-10.0 0.0 0.0];
    params.leftArmInit  = [ -19.7  29.7  0.0  44.9  0.0]';%params.leftArmInit = zeros(size(params.leftArmInit));
    params.rightArmInit = [ -19.7  29.7  0.0  44.9  0.0]';%params.rightArmInit = zeros(size(params.rightArmInit));
    params.leftLegInit  = [ 25.5   0.1  0.0 -38.5 -5.5 -0.1]';%params.leftLegInit = zeros(size(params.leftLegInit));
    params.rightLegInit = [ 25.5   0.1  0.0 -38.5 -5.5 -0.1]';%params.rightLegInit = zeros(size(params.rightLegInit));

    params.qjInit = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
else
    % random pose within joint limits
    [jl1,jl2] = wbm_jointLimits();
    params.qjInit = jl1 + rand(25,1).*(jl2-jl1);
end

params.dqjInit = zeros(params.ndof,1);
params.dx_bInit = zeros(3,1);
params.omega_bInit = zeros(3,1);
params.dampingCoeff = 0.00;

[pos,rot] = wbm_getWorldFrameFromFixLnk('l_sole',params.qjInit);
% fprintf('Converting to a set world frame... \n');
wbm_setWorldFrame(rot,pos,[ 0,0,-9.81]');

[T_b,~,~,~] = wbm_getState();

fprintf('Prior rotation \n');
disp(rot);

fprintf('Prior rotation check (R^T*R)\n');
disp(rot'*rot);

wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));

[T_b_Got,~,~,~] = wbm_getState();
[posGot,rotGot] = WBM.utilities.frame2posRotm(T_b_Got);

fprintf('Post convertion rotation \n');
disp(rotGot);
%fprintf('Post position \n');
%disp(posGot);

WBMAssertEqual(posGot,pos);
WBMAssertEqual(rotGot,rot);
% fprintf('Post conversion rotation check (R^T*R)\n');
% disp(rotGot'*rotGot);
WBMAssertEqual(rotGot'*rotGot,eye(3,3));
% fprintf('Post conversion frame \n');
% disp(T_b_Got');
WBMAssertEqual(T_b_Got,T_b);
% fprintf('Post conversion quaternion norm \n');
% disp(norm(T_b_Got(4:end)));
WBMAssertEqual(norm(T_b_Got(4:end)),1);
