function gainsOpt = constrGainsInit(gainsFromKronecher,linearization,CONFIG)

%% Initial conditions
ndof          = CONFIG.ndof;
gainsOpt      = gainsFromKronecher;
vImp          = log(diag(CONFIG.gainsInit.impedances));
vDamp         = log(diag(CONFIG.gainsInit.dampings));
vMom          = log(diag(CONFIG.gainsInit.MomentumGains));
vIntM         = log(diag(CONFIG.gainsInit.intMomentumGains));
R1ini         = eye(6); 
R2ini         = eye(ndof);
R3ini         = eye(6);
R4ini         = eye(ndof);
vR1ini        = R1ini(:);
vR2ini        = R2ini(:);
vR3ini        = R3ini(:);
vR4ini        = R4ini(:);
gainsVectInit = [vIntM;vR1ini;vImp;vR2ini;vMom;vR3ini;vDamp;vR4ini];

%% Gains integrator
CONFIG.wait     = waitbar(0,'Gains integration...');
gainsVectOpt    = integrateGains(gainsVectInit,gainsFromKronecher,CONFIG);
delete(CONFIG.wait)

%% Optimized gains
gainsVectOptEnd = gainsVectOpt(:,end);

vL1 = gainsVectOptEnd(1:6);
vR1 = gainsVectOptEnd(7:42);
vL2 = gainsVectOptEnd(43:67);
vR2 = gainsVectOptEnd(68:692);
vL3 = gainsVectOptEnd(693:698);
vR3 = gainsVectOptEnd(699:734);
vL4 = gainsVectOptEnd(735:759);
vR4 = gainsVectOptEnd(760:1384);

L1  = diag(vL1);
R1  = reshape(vR1,[6,6]);
L2  = diag(vL2);
R2  = reshape(vR2,[ndof,ndof]);
L3  = diag(vL3);
R3  = reshape(vR3,[6,6]);
L4  = diag(vL4);
R4  = reshape(vR4,[ndof,ndof]);

gainsOpt.intMomentumGains       = R1'*expm(L1)*R1;
gainsOpt.impedances             = R2'*expm(L2)*R2; 
gainsOpt.MomentumGains          = R3'*expm(L3)*R3;
gainsOpt.dampings               = R4'*expm(L4)*R4;

%% Visualization
ACartesian            = linearization.ACartesian;
BCartesian            = linearization.BCartesian;
ANull                 = linearization.ANull;
BNull                 = linearization.BNull;
gainsOpt.KSn          = ACartesian*gainsOpt.intMomentumGains*BCartesian + ANull*gainsOpt.impedances*BNull;
gainsOpt.KDn          = ACartesian*gainsOpt.MomentumGains*BCartesian + ANull*gainsOpt.dampings*BNull;

figure
image(gainsOpt.KSn,'CDataMapping','scaled')
colorbar
figure
image(gainsOpt.KDn,'CDataMapping','scaled')
colorbar

AStateOpt             = [zeros(ndof), eye(ndof); -gainsOpt.KSn -gainsOpt.KDn];

figure(20)
plot(real(eig(AStateOpt)),imag(eig(AStateOpt)),'ob')

end
