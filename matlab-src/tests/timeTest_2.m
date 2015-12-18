addpath('./../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../build');
%Running a 10,000 run trial
totTime = 0;numRuns = 1000;

refLink1 = 'r_sole';
refLink2 = 'l_gripper';


fprintf('Starting normal mode trial \n-------------------------- \n');

tic;
%wbm_modelInitialise('icubGazeboSim');
wbm_modelInitialise();
initTime = toc();

fprintf('Initialisation time : %e secs\n',initTime);
fprintf('Num Trials : %d \n Starting Trial...\n',numRuns);
tic;

for i = 1:numRuns
    
    %Setting State to random values
    q = rand(25,1);dq = rand(25,1);dxb = rand(6,1);R = eye(3);p = rand(3,1);g = [0;0;9.8];
    wbm_setWorldFrame(R,p,g);
    
    %Mex-WholeBodyModel Components
    M = wbm_massMatrix(R,p,q);    
    H = wbm_generalisedBiasForces(R,p,q,dq,dxb);    
    H = wbm_generalisedBiasForces(R,p,q,dq,dxb); 
    H = wbm_generalisedBiasForces(R,p,q,dq,dxb); 
    DjDq1 = wbm_djdq(R,p,q,dq,dxb,refLink1);
    DjDq2 = wbm_djdq(R,p,q,dq,dxb,refLink2);    
    J = wbm_jacobian(R,p,q,refLink1);J = wbm_jacobian(R,p,q,refLink2);
    J = wbm_jacobian(R,p,q,refLink1);J = wbm_jacobian(R,p,q,refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Normal-Mode Trial Total Time : %f secs \n',totTime);
fprintf('Normal-Mode Trial Average Time : %e secs\n',totTime/numRuns);

clear all;

fprintf('\n\nStarting optimised mode trial \n-------------------------- \n');

totTime = 0;numRuns = 1000;
tic;
wbm_modelInitialise('icubGazeboSim');
%wbm_modelInitialise();
initTime = toc();
fprintf('Initialisation time : %e secs \n Starting Trial...\n ',initTime);

refLink1 = 'r_sole';
refLink2 = 'l_gripper';

tic;
for i = 1:numRuns
    
    %Setting State to random values
    q = rand(25,1);dq = rand(25,1);dxb = rand(6,1); R = eye(3); p =rand(3,1);g = [0;0;9.8];
    
    wbm_updateState(q,dq,dxb);
    wbm_setWorldFrame(R,p,g);
    
    %Mex-WholeBodyModel Components
    M = wbm_massMatrix();    
    H = wbm_generalisedBiasForces();    
    H = wbm_generalisedBiasForces();  
    H = wbm_generalisedBiasForces();  
    DjDq1 = wbm_djdq(refLink1);
    DjDq2 = wbm_djdq(refLink2);
    J = wbm_jacobian(refLink1);J = wbm_jacobian(refLink2);
    J = wbm_jacobian(refLink1);J = wbm_jacobian(refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Optimised-Mode Trial Total Time : %f secs\n',totTime);

fprintf('Optimised-Mode Trial Average Time : %e secs \n',totTime/numRuns);
