%Running a 10,000 run trial
totTime = 0;numRuns = 10000;

refLink1 = 'r_sole';tic;
refLink2 = 'l_gripper';

disp('Num Trials :');disp(numRuns);
for i = 1:numRuns
    
    %Setting State to random values
    q = rand(32,1);dq = rand(32,1);dxb = rand(6,1);
    
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
disp('Total Time : ');disp(totTime);
disp('Average Time :');disp(totTime/numRuns);
