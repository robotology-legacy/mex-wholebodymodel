function gainsOpt = constrGainsInit(gainsInit,linearization,CONFIG)

%% initial conditions
gainsOpt  = gainsInit;

vImp   = log(diag(CONFIG.gainsInit.impedances));
vDamp  = log(diag(CONFIG.gainsInit.dampings));
vMom   = log(diag(CONFIG.gainsInit.MomentumGains));
vIntM  = log(diag(CONFIG.gainsInit.intMomentumGains));
R1ini  = eye(6); 
R2ini  = eye(25);
R3ini  = eye(6);
R4ini  = eye(25);
vR1ini = R1ini(:);
vR2ini = R2ini(:);
vR3ini = R3ini(:);
vR4ini = R4ini(:);

gainsVectInit = [vIntM;vR1ini;vImp;vR2ini;vMom;vR3ini;vDamp;vR4ini];

%% integrator
CONFIG.wait     = waitbar(0,'Gains integration...');
gainsVectOpt    = integrateGains(gainsVectInit,gainsInit,CONFIG);
delete(CONFIG.wait)

%% Optimized gains
gainsVectOptFin = gainsVectOpt(:,end);

vL1 = gainsVectOptFin(1:6);
vR1 = gainsVectOptFin(7:42);
vL2 = gainsVectOptFin(43:67);
vR2 = gainsVectOptFin(68:692);
vL3 = gainsVectOptFin(693:698);
vR3 = gainsVectOptFin(699:734);
vL4 = gainsVectOptFin(735:759);
vR4 = gainsVectOptFin(760:1384);

L1  = diag(vL1);
R1  = reshape(vR1,[6,6]);
L2  = diag(vL2);
R2  = reshape(vR2,[25,25]);
L3  = diag(vL3);
R3  = reshape(vR3,[6,6]);
L4  = diag(vL4);
R4  = reshape(vR4,[25,25]);

gainsOpt.intMomentumGains       = R1'*expm(L1)*R1;
gainsOpt.impedances             = R2'*expm(L2)*R2; 
gainsOpt.MomentumGains          = R3'*expm(L3)*R3;
gainsOpt.dampings               = R4'*expm(L4)*R4;

%% visualization
figure
surf(gainsOpt.impedances)
figure
surf(gainsOpt.dampings)
figure
surf(gainsOpt.MomentumGains)
figure
surf(gainsOpt.intMomentumGains)

ACartesian            = linearization.ACartesian;
BCartesian            = linearization.BCartesian;
ANull                 = linearization.ANull;
BNull                 = linearization.BNull;

gainsOpt.KSn = ACartesian*gainsOpt.intMomentumGains*BCartesian + ANull*gainsOpt.impedances*BNull;
gainsOpt.KDn = ACartesian*gainsOpt.MomentumGains*BCartesian + ANull*gainsOpt.dampings*BNull;

ndof  = CONFIG.ndof;
State = [zeros(ndof), eye(ndof); -gainsOpt.KSn -gainsOpt.KDn];

disp('Linearized and constrained system eigenvalues')
disp(eig(State))

figure
surf(gainsOpt.KSn)
figure
surf(gainsOpt.KDn)

end
