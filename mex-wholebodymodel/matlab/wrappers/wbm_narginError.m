function wbm_narginError(func_name)
    % WBM_NARGINERROR throws an error and display an error message that the NARGIN is incorrect.
    %
    %   INPUT ARGUMENTS:  func_name -- name of the current function (string).
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    errmsg = sprintf('%s Check the documentations!', WBM.wbmErrorMsg.WRONG_NARGIN);
    error('%s: %s\n', func_name, errmsg);
end
