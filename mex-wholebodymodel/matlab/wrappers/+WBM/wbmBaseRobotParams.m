classdef wbmBaseRobotParams
    properties
        robot_model@WBM.wbmBaseRobotModel
        robot_config@WBM.wbmBaseRobotConfig
        wf2fixLnk@logical scalar
    end
end
