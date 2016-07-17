fprintf('\nMex_wbm\n');

installDir = '/home/gnava/codyco-superbuild/build/install';
mexDir     = [installDir, filesep, 'mex'];
mexWbiUt       = [mexDir, filesep, 'mexwbi-utilities'];
mexWbiWrap     = [mexDir, filesep, 'mexwbi-wrappers'];

if exist(mexDir, 'dir')
    addpath(mexDir);
end

if exist(mexWbiUt, 'dir')
    addpath(mexWbiUt);
end

if exist(mexWbiWrap, 'dir')
    addpath(mexWbiWrap);
end

fileDir = userpath;
pathSeparatorLocation = strfind(fileDir, pathsep);

if isempty(fileDir)
    error('Empty userpath. Please set the userpath before running this script');
elseif size(pathSeparatorLocation, 2) > 1
    error('Multiple userpaths. Please set a single userpath before running this script');
end

if (~isempty(pathSeparatorLocation))
    fileDir(pathSeparatorLocation) = [];
end

fprintf('Saving paths to %s\n\n', [fileDir, filesep, 'pathdef.m']);

if (~savepath([fileDir, filesep, 'pathdef.m']))
    fprintf(['A file called pathdef.m has been created in your %s folder.\n', ...
        'This should be enough to permanentely add Mex-wbm to ', ...
        'your MATLAB installation.\n'], fileDir);
else
    disp('There was an error generating pathdef.m To proceed please manually add the contents of variables mexDir, mexWbiWrap and mexWbiWrap to your matlabpath');
end
