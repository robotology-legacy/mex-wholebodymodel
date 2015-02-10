
addpath('./../build');
%Running a 10,000 run trial
totTime = 0;numRuns = 10000;

refLink1 = 'r_sole';
refLink2 = 'l_gripper';


fprintf('Starting normal mode trial \n-------------------------- \n');

tic;
wholeBodyModel('model-initialise','icubGazeboSim');
initTime = toc();

fprintf('Initialisation time : %e secs\n',initTime);
fprintf('Num Trials : %d \n Starting Trial...\n',numRuns);
tic;

for i = 1:numRuns
    
    %Setting State to random values
    q = rand(25,1);dq = rand(25,1);dxb = rand(6,1);
    
    %Mex-WholeBodyModel Components
    M = wholeBodyModel('mass-matrix',q);    
    H = wholeBodyModel('generalised-forces',q,dq,dxb);    
    DjDq1 = wholeBodyModel('djdq',q,dq,dxb,refLink1);
    DjDq2 = wholeBodyModel('djdq',q,dq,dxb,refLink2);    
    J = wholeBodyModel('jacobian',q,refLink1);J = wholeBodyModel('jacobian',q,refLink2);
    J = wholeBodyModel('jacobian',q,refLink1);J = wholeBodyModel('jacobian',q,refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Normal-Mode Trial Total Time : %f secs \n',totTime);
fprintf('Normal-Mode Trial Average Time : %e secs\n',totTime/numRuns);

clear all;

fprintf('\n\nStarting optimised mode trial \n-------------------------- \n');

totTime = 0;numRuns = 10000;
tic;
wholeBodyModel('model-initialise','icub');
initTime = toc();
fprintf('Initialisation time : %e secs \n Starting Trial...\n ',initTime);

refLink1 = 'r_sole';
refLink2 = 'l_gripper';

tic;
for i = 1:numRuns
    
    %Setting State to random values
    q = rand(25,1);dq = rand(25,1);dxb = rand(6,1);
    
    wholeBodyModel('update-state',q,dq,dxb);
    
    %Mex-WholeBodyModel Components
    M = wholeBodyModel('mass-matrix');    
    H = wholeBodyModel('generalised-forces');    
    DjDq1 = wholeBodyModel('djdq',refLink1);
    DjDq2 = wholeBodyModel('djdq',refLink2);    
    J = wholeBodyModel('jacobian',refLink1);J = wholeBodyModel('jacobian',refLink2);
    J = wholeBodyModel('jacobian',refLink1);J = wholeBodyModel('jacobian',refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Optimised-Mode Trial Total Time : %f secs\n',totTime);
fprintf('Optimised-Mode Trial Average Time : %e secs \n',totTime/numRuns);
