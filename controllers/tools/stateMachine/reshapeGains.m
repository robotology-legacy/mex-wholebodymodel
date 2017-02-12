function gains = reshapeGains(gainVect,CONFIG)

gains = CONFIG.gainsInit;

gainsPCoM   = diag(gainVect(1:3));
gainsDCoM   = 2*sqrt(gainsPCoM);

gains.impedances  = diag(gainVect(4:end));
gains.dampings    = 2*sqrt(gains.impedances);

gains.momentumGains      = [gainsDCoM zeros(3); zeros(3) gains.momentumGains(4:end,4:end)];
gains.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) gains.intMomentumGains(4:end,4:end)];

end
