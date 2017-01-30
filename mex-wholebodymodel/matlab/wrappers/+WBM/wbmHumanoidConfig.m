classdef wbmHumanoidConfig < WBM.wbmBaseRobotConfig
    properties
        jpos_head@double       vector
        jpos_torso@double      vector
        jpos_left_arm@double   vector 
        jpos_left_hand@double  vector
        jpos_left_leg@double   vector
        jpos_left_feet@double  vector
        jpos_right_arm@double  vector
        jpos_right_hand@double vector
        jpos_right_leg@double  vector
        jpos_right_feet@double vector
    end
end
