classdef wbmRobotParams < handle
    properties
        model@WBM.wbmRobotModel
        config@WBM.wbmRobotConfig
        wf2fixlnk@logical scalar
    end

    methods(Sealed)
        function newObj = copy(obj)
            newObj = WBM.utilities.copyObj(obj);
        end

    end
end
