function [dotgainsVect] = gainConstrAll(t,gainsVect,gainsKron,CONFIG)

%% Generate the initial conditions
waitbar(t/CONFIG.tEndgain,CONFIG.wait)

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

KL1 = 20;
KO1 = 20;
KL2 = 20;
KO2 = 20;
KL3 = 20;
KO3 = 20;
KL4 = 20;
KO4 = 20;

%% Apply constraints
[dotL1,dotR1,~,~,~]        =  gainConstr(L1,R1,gainsKron.intMomentumGains,zeros(6),KL1,KO1);
[dotL2,dotR2,~,~,~]        =  gainConstr(L2,R2,gainsKron.impedances,zeros(CONFIG.ndof),KL2,KO2);
[dotL3,dotR3,~,~,~]        =  gainConstr(L3,R3,gainsKron.MomentumGains,zeros(6),KL3,KO3);
[dotL4,dotR4,~,~,~]        =  gainConstr(L4,R4,gainsKron.dampings,zeros(CONFIG.ndof),KL4,KO4);

% [dotL1,dotR1,dotL2,dotR2,~,~,~,~]   ...
%           = gainConstrAndOpt(L1,R1,L2,R2,CONFIG.linearization.KSdes,zeros(CONFIG.ndof),KL1,KO1,KL2,KO2,CONFIG.linearization.ACartesian,...
%           CONFIG.linearization.BCartesian,CONFIG.linearization.ANull,CONFIG.linearization.BNull);
% [dotL3,dotR3,dotL4,dotR4,~,~,~,~]   ...
%           = gainConstrAndOpt(L3,R3,L4,R4,CONFIG.linearization.KDdes,zeros(CONFIG.ndof),KL3,KO3,KL4,KO4,CONFIG.linearization.ACartesian,...
%           CONFIG.linearization.BCartesian,CONFIG.linearization.ANull,CONFIG.linearization.BNull);

%% Results
vdotR1 = dotR1(:);
vdotR2 = dotR2(:);
vdotR3 = dotR3(:);
vdotR4 = dotR4(:);

dotgainsVect = [dotL1;vdotR1;dotL2;vdotR2;dotL3;vdotR3;dotL4;vdotR4];

end
