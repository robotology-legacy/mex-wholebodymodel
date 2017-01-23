function [gainsVectdot] = gainsDynamics(t,gainsVect,gainsKronecker,CONFIG)
%% Generate the initial conditions
waitbar(t/CONFIG.tEndGain,CONFIG.wait)

vL1 = gainsVect(1:6);
vR1 = gainsVect(7:42);
vL2 = gainsVect(43:67);
vR2 = gainsVect(68:692);
vL3 = gainsVect(693:698);
vR3 = gainsVect(699:734);
vL4 = gainsVect(735:759);
vR4 = gainsVect(760:1384);

L1  = vL1;
R1  = reshape(vR1,[6,6]);
L2  = vL2;
R2  = reshape(vR2,[25,25]);
L3  = vL3;
R3  = reshape(vR3,[6,6]);
L4  = vL4;
R4  = reshape(vR4,[25,25]);

KL1 = 65;
KO1 = 65;
KL2 = 65;
KO2 = 65;
KL3 = 65;
KO3 = 65;
KL4 = 65;
KO4 = 65;

%% Apply constraints
[dotL1,dotR1,~,~,~]        =  gainsDerivative(L1,R1,(gainsKronecker.intMomentumGains+gainsKronecker.intMomentumGains')/2,zeros(6),KL1,KO1);
[dotL2,dotR2,~,~,~]        =  gainsDerivative(L2,R2,(gainsKronecker.impedances+gainsKronecker.impedances')/2,zeros(CONFIG.ndof),KL2,KO2);
[dotL3,dotR3,~,~,~]        =  gainsDerivative(L3,R3,(gainsKronecker.momentumGains+gainsKronecker.momentumGains')/2,zeros(6),KL3,KO3);
[dotL4,dotR4,~,~,~]        =  gainsDerivative(L4,R4,(gainsKronecker.dampings+gainsKronecker.dampings')/2,zeros(CONFIG.ndof),KL4,KO4);

%% Results
vdotR1 = dotR1(:);
vdotR2 = dotR2(:);
vdotR3 = dotR3(:);
vdotR4 = dotR4(:);

gainsVectdot = [dotL1;vdotR1;dotL2;vdotR2;dotL3;vdotR3;dotL4;vdotR4];

end
