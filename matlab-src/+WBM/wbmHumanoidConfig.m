classdef wbmHumanoidConfig < WBM.wbmBaseRobotConfig
    properties
        pos_head@double      vector
        pos_torso@double     vector
        pos_leftArm@double   vector 
        pos_leftHand@double  vector
        pos_leftLeg@double   vector
        pos_leftFeet@double  vector
        pos_rightArm@double  vector
        pos_rightHand@double vector
        pos_rightLeg@double  vector
        pos_rightFeet@double vector
    end
end