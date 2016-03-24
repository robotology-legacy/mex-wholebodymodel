function nonlinearImpedances = nonLinImp(qjDes, qj, impedences_at_0, increasingRatesImp, qTildeMax)
%% nonlinearImpedances 
%  Corrects the impedances at joints by adding a nonlinear term. 
%  The output is the vector of nonlinear impedances [ndofx1]
qTilde = qj-qjDes;
            
nonlinearImpedances = (impedences_at_0 + increasingRatesImp.*(tan(pi.*qTilde./(2*qTildeMax)).^2).')';
    
end