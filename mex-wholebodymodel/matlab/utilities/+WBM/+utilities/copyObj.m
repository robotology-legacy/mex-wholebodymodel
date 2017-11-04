function newObj = copyObj(obj)
    % Copy function: Replacement for the "matlab.mixin.Copyable.copy()" to create deep object copies.
    % Source: <http://undocumentedmatlab.com/blog/general-use-object-copy>
    try
        % Matlab-tuning: Try to use directly the memory (faster).
        % Note: This works only for R2010b or newer.
        objByteArray = getByteStreamFromArray(obj);
        newObj = getArrayFromByteStream(objByteArray);
    catch
        % else, for R2010a or earlier, serialize via a
        % temporary file (slower).
        fname = [tempname '.mat'];
        save(fname, 'obj');
        newObj = load(fname);
        newObj = newObj.obj;
        delete(fname);
    end
end
