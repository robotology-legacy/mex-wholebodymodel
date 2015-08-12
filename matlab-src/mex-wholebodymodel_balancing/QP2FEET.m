function desiredf0 = QP2FEET(HessianMatrixQP2Feet,gradientQP2Feet,...
                             ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet)

% this function computes the quadratic programming for the robot standing
% on two feet. the result is the vector f0 that minimizes joint torques. 

%   [desiredf0,~,exitFlag,iter,lambda,auxOutput] = qpOASES(HessianMatrixQP2Feet,gradientQP2Feet',ConstraintsMatrixQP2Feet,...
%                                                          [],[],[],bVectorConstraintsQp2Feet');           
    [desiredf0,~,exitFlag,iter,lambda,auxOutput] = qpOASES(HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,...
                                                           [],[],[],bVectorConstraintsQp2Feet);           
            
    if exitFlag ~= 0

         disp('QP failed');
         disp(exitFlag);
         disp(iter);
         disp(auxOutput);
         disp(lambda);
%         desiredf0 = zeros(6*2+3,1);
         error('qp_failed')
    end

      