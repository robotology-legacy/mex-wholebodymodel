%addpath('./../build');
%Running a 10,000 run trial
totTime = 0;numRuns = 10000;

refLink1 = 'r_hip_1';
refLink2 = 'l_hip_1';

% number of joint for iCubGenova03
nj       = 25;   
% number of joint for iCubHeidelberg01
% nj       = 15;   


fprintf('Starting normal mode trial \n-------------------------- \n');

tic;
wholeBodyModel('model-initialise');
initTime = toc();

fprintf('Initialisation time : %e secs\n',initTime);
fprintf('Num Trials : %d \n Starting Trial...\n',numRuns);
tic;

qj = rand(nj,1);
xTb = wholeBodyModel('forward-kinematics',qj,refLink1);

for i = 1:numRuns
    
    %Setting State to random values
    qj = rand(nj,1);dqj = rand(nj,1);dxb = rand(6,1);
    
    %Mex-WholeBodyModel Components
    M = wholeBodyModel('mass-matrix',qj);    
    H = wholeBodyModel('generalised-forces',qj,dqj,dxb);    
    DjDq1 = wholeBodyModel('djdq',qj,dqj,dxb,refLink1);
    DjDq2 = wholeBodyModel('djdq',qj,dqj,dxb,refLink2);    
    J1 = wholeBodyModel('jacobian',qj,refLink1);J2 = wholeBodyModel('jacobian',qj,refLink2);
    J3 = wholeBodyModel('jacobian',qj,refLink1);J4 = wholeBodyModel('jacobian',qj,refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Normal-Mode Trial Total Time : %f secs \n',totTime);
fprintf('Normal-Mode Trial Average Time : %e secs\n',totTime/numRuns);

fprintf('\n\nStarting optimised mode trial \n-------------------------- \n');

totTime = 0;numRuns = 10000;
tic;
wholeBodyModel('model-initialise');
initTime = toc();
fprintf('Initialisation time : %e secs \n Starting Trial...\n ',initTime);

refLink1 = 'r_hip_1';
refLink2 = 'l_hip_1';

tic;
for i = 1:numRuns
    
    %Setting State to random values
    qj = rand(nj,1);dqj = rand(nj,1);dxb = rand(6,1);
    
    wholeBodyModel('update-state',qj,dqj,dxb);
    
    %Mex-WholeBodyModel Components
    M = wholeBodyModel('mass-matrix');    
    H = wholeBodyModel('generalised-forces');    
    DjDq1 = wholeBodyModel('djdq',refLink1);
    DjDq2 = wholeBodyModel('djdq',refLink2);    
    J1 = wholeBodyModel('jacobian',refLink1);J2 = wholeBodyModel('jacobian',refLink2);
    J3 = wholeBodyModel('jacobian',refLink1);J4 = wholeBodyModel('jacobian',refLink2);
end
totTime = toc();
%Benchmarks
fprintf('Optimised-Mode Trial Total Time : %f secs\n',totTime);
fprintf('Optimised-Mode Trial Average Time : %e secs \n',totTime/numRuns);
