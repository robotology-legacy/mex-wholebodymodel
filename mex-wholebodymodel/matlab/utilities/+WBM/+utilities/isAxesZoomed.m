function result = isAxesZoomed(hax)
    % Determine the axes zoom state.
    % Sources: <https://undocumentedmatlab.com/blog/determining-axes-zoom-state>
    if ~nargin
        hax = gca;
    end
    result = false;

    orig_info = getappdata(hax, 'matlab_graphics_resetplotview');
    if ( isempty(orig_info) || ...
         (isequal(get(hax, 'XLim'), orig_info.XLim) && ...
          isequal(get(hax, 'YLim'), orig_info.YLim) && ...
          isequal(get(hax, 'ZLim'), orig_info.ZLim)) )
        return
    end
    % else, the axes has been zoomed ...
    result = true;
end
