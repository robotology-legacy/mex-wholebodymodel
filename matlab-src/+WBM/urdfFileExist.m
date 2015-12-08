function fex = urdfFileExist(urdf_file_name)
	fex = false;

 	dir_listing = dir(urdf_file_name);
 	% check if the file name a folder ...
    if (length(dir_listing) == 1)
    	% it is not a folder ...
        fex = ~dir_urdf_file.isdir;
    end
end
