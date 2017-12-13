classdef wbmTargetPoint < WBM.wbmGObj
    properties
        pos@double        = zeros(3,1);
        description@char  = '';

        line_width@double scalar = 0.5;
        marker@char       = 'o'; % circle
        mkr_size@double   scalar = 8;
        mkr_color         = 'yellow';
    end

    methods
        function obj = wbmTargetPoint(p)
            switch nargin
                case 0
                    return
                case 1
                    WBM.utilities.chkfun.checkCVecDim(p, 3, 'wbmTargetPoint::wbmTargetPoint');
                    obj.pos = p;
                otherwise
                    error('wbmTargetPoint::wbmTargetPoint: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function hgo = getGObj(obj)
            % mark the target point to be reached:
            p = obj.pos;
            hold on;
            hgo = plot3(p(1,1), p(2,1), p(3,1), 'LineStyle', 'none', 'LineWidth', obj.line_width, ...
                        'Marker', obj.marker, 'MarkerSize', obj.mkr_size, 'MarkerEdgeColor', obj.mkr_color);
        end

        function hgo = updGObj(obj, hgo)
            % update the target point:
            hgo.XData = obj.pos(1,1);
            hgo.YData = obj.pos(2,1);
            hgo.ZData = obj.pos(3,1);
        end

        function obj = set.pos(obj, p)
            WBM.utilities.chkfun.checkCVecDim(p, 3, 'wbmTargetPoint::set.pos');
            obj.pos = p;
        end

    end
end
