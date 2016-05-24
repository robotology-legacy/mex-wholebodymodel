classdef wbmRobotBaseParams
    properties
        robot_model@WBM.wbmBaseModelParams;
        robot_config@WBM.wbmBaseRobotConfig;
        wf2FixLnk@logical scalar;
    end
end
