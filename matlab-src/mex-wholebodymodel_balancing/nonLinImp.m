function nonlinearImpedences = nonLinImp(qDes,q,qMin,qMax,impedences_at_0,increasingRatesImp,qTildeMax)
 
% this function computes the correction of impedances at joints to add a
% nonlinear term.

%#codegen
%   tp =  pi./(2*(qMax-qDes));
%   tn = -pi./(2*(qMin-qDes));

    qTilde = q - qDes;
            
    nonlinearImpedences = (impedences_at_0   + increasingRatesImp.*(tan(pi.*qTilde./(2*qTildeMax)).^2).')';
%   nonlinearImpedences = (impedences_at_0   + increasingRatesImp.*(tan(pi.*qTilde./(2*qTildeMax)).^2))';

%                                              increasingRatesImp.*tan((tp.*stepFcn(qTilde) + tn.*stepFcn(-qTilde)).*qTilde).^2)';
                                           
end


% function y = stepFcn(x) 
%    %#codegen
%    y = x > 0; 
% end