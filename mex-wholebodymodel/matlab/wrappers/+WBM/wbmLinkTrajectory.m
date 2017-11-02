classdef wbmLinkTrajectory
    properties
        urdf_link_name@char   = 'none';
        jnt_annot_pos@cell    vector = {}; % {lnk_group_name, idx}
        description@char      = '';

        line_style@char       = ':'; % dotted line
        line_width@double     scalar = 0.5;
        line_color            = 'green';

        ept_marker@char       = 'o'; % circle
        ept_marker_sz@double  scalar = 8;
        ept_color             = 'green';

        lnk_pos@double matrix = [];
    end

    methods
        function obj = wbmLinkTrajectory(lnk_name, lnk_pos)
            switch nargin
                case 0
                    return
                case 2
                    if (size(lnk_pos,2) ~= 3)
                        error('wbmLinkTrajectory::wbmLinkTrajectory: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                    end
                    obj.lnk_pos = lnk_pos;
            end
            % if nargin >= 1:
            WBM.utilities.chkfun.checkLinkName(lnk_name, 'wbmLinkTrajectory::wbmLinkTrajectory');
            obj.urdf_link_name = lnk_name;
        end

        function hgo = getGObj(obj)
            dpts = obj.lnk_pos;
            hgo  = gobjects(2,1);

            hgo(1,1) = plot3(dpts(:,1), dpts(:,2), dpts(:,3), 'LineStyle', obj.line_style, ...
                             'LineWidth', obj.line_width, 'Color', obj.line_color);
            hold on;
            % mark the endpoint ...
            hgo(2,1) = plot3(dpts(end,1), dpts(end,2), dpts(end,3), 'Marker', obj.ept_marker, ...
                             'MarkerSize', obj.ept_marker_sz, 'MarkerEdgeColor', obj.ept_color);
        end

        function hgo = updGObj(obj, hgo)
            htl = hgo(1,1);
            hep = hgo(2,1);

            % update trajectory line:
            htl.XData = obj.lnk_pos(:,1);
            htl.YData = obj.lnk_pos(:,2);
            htl.ZData = obj.lnk_pos(:,3);
            % update endpoint:
            hep.XData = obj.lnk_pos(end,1);
            hep.YData = obj.lnk_pos(end,2);
            hep.ZData = obj.lnk_pos(end,3);

            hgo(1,1) = htl;
            hgo(2,1) = hep;
        end

        function obj = set.urdf_link_name(obj, lnk_name)
            WBM.utilities.chkfun.checkLinkName(lnk_name, 'wbmLinkTrajectory::set.urdf_link_name');
            obj.urdf_link_name = lnk_name;
        end

        function obj = set.jnt_annot_pos(obj, annot_pos)
            WBM.utilities.chkfun.checkRVecDim(annot_pos, 2, 'wbmLinkTrajectory::set.jnt_annot_pos');
            if ( ~iscell(annot_pos) || ~ischar(annot_pos{1,1}) || ~isreal(annot_pos{1,2}) )
                error('wbmLinkTrajectory::set.jnt_annot_pos: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            annot_pos{1,2} = uint8(annot_pos{1,2});
            obj.jnt_annot_pos = annot_pos; % {lnk_group_name, idx}
        end

    end
end
