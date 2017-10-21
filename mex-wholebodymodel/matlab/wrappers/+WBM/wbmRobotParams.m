classdef wbmRobotParams < handle
    properties
        model@WBM.wbmRobotModel
        config@WBM.wbmRobotConfig
        wf2fixlnk@logical scalar
    end

    methods(Sealed)
        % Copy function: Replacement for the "matlab.mixin.Copyable.copy()" to create deep object copies.
        % Source: <http://undocumentedmatlab.com/blog/general-use-object-copy>
        function newObj = copy(obj)
            try
                % Matlab R2010b or newer:
                % try to use directly the memory (faster) ...
                objByteArray = getByteStreamFromArray(obj);
                newObj = getArrayFromByteStream(objByteArray);
            catch
                % Matlab R2010a or earlier:
                % serialize via a temporary file (slower) ...
                fname = [tempname '.mat'];
                save(fname, 'obj');
                newObj = load(fname);
                newObj = newObj.obj;
                delete(fname);
            end
        end

    end
end
