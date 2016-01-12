function nonlinearImpedences = nonLinImp(qjDes, qj, impedences_at_0, increasingRatesImp, qTildeMax)
%% nonlinearImpedances 
% corrects the impedances at joints with a nonlinear term
qTilde = qj-qjDes;
            
nonlinearImpedences = (impedences_at_0 + increasingRatesImp.*(tan(pi.*qTilde./(2*qTildeMax)).^2).')';
    
end