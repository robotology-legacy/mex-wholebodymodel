Basic Whole-Body Model Class
----------------------------

.. autoclass:: WBMBase
   :show-inheritance:

   .. automethod:: WBMBase.WBMBase
   .. automethod:: WBMBase.copy
   .. automethod:: WBMBase.delete
   .. automethod:: WBMBase.initModel
   .. automethod:: WBMBase.initModelURDF
   .. automethod:: WBMBase.setWorldFrame
   .. automethod:: WBMBase.setInitWorldFrame
   .. automethod:: WBMBase.getWorldFrameFromFixLnk
   .. automethod:: WBMBase.getWorldFrameFromDfltFixLnk
   .. automethod:: WBMBase.setState
   .. automethod:: WBMBase.getState
   .. automethod:: WBMBase.getFloatingBaseState
   .. automethod:: WBMBase.transformationMatrix
   .. automethod:: WBMBase.massMatrix
   .. automethod:: WBMBase.getJointLimits
   .. automethod:: WBMBase.isJointLimit
   .. automethod:: WBMBase.generalizedBaseAcc
   .. automethod:: WBMBase.inverseDynamics
   .. automethod:: WBMBase.jacobian
   .. automethod:: WBMBase.dJdq
   .. automethod:: WBMBase.centroidalMomentum
   .. automethod:: WBMBase.forwardKinematics
   .. automethod:: WBMBase.generalizedBiasForces
   .. automethod:: WBMBase.generalizedForces
   .. automethod:: WBMBase.coriolisBiasForces
   .. automethod:: WBMBase.gravityBiasForces
   .. automethod:: WBMBase.frictionForces
   .. automethod:: WBMBase.wholeBodyDynamics
   .. automethod:: WBMBase.dispModel

   .. _yarpWholeBodyInterface: https://github.com/robotology/yarp-wholebodyinterface
   .. _mex-wholeBodyModel: https://github.com/robotology/mex-wholebodymodel
