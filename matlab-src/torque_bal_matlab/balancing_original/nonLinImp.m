function nonlinearImpedences = nonLinImp(qDes, q, impedences_at_0,increasingRatesImp, qTildeMax)
%% nonlinearImpedances 
% corrects the impedances at joints with a nonlinear term
qTilde = q-qDes;
            
nonlinearImpedences = (impedences_at_0   + increasingRatesImp.*(tan(pi.*qTilde./(2*qTildeMax)).^2).')';
    
end