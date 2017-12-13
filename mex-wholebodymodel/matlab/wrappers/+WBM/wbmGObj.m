classdef (Abstract) wbmGObj
    properties(Abstract)
        description@char         % annotation of the graphic object
        line_width@double scalar
    end

    methods(Abstract)
        hgo = getGObj(obj)      % get the graphic object handle
        hgo = updGObj(obj, hgo) % update the graphic object handle
    end
end