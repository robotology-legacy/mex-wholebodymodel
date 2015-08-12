function desiredf0 = QP1FOOT(HessianMatrixQP1Foot,gradientQP1Foot,...
                             ConstraintsMatrixQP1Foot,bVectorConstraintsQP1Foot)
     
% this function computes the quadratic programming for the robot standing
% on one foot. the result is the vector f0 of desired contact forces. 

% [desiredf0,~,exitFlag,iter,lambda,auxOutput] = qpOASES(HessianMatrixQP1Foot,gradientQP1Foot',....
%                                                        ConstraintsMatrixQP1Foot,[],[],[],bVectorConstraintsQP1Foot');
  [desiredf0,~,exitFlag,iter,lambda,auxOutput] = qpOASES(HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,...
                                                         [],[],[],bVectorConstraintsQP1Foot);           

  if exitFlag ~= 0
      
  disp('QP failed with');
  disp( exitFlag)
  disp( iter)
  disp( auxOutput)
  disp( lambda)
% desiredf0 = zeros(6+3,1);
  error('qp_failed')
      
  end

   
