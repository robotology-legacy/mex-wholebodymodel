function result = isHomog(tform, epsilon)
    WBM.utilities.checkMatDim(tform, 4, 4, 'isHomog');

    if ~exist('epsilon', 'var')
        epsilon = 1e-12; % min. value to treat a number as zero ...
    end

    result = false;
    if (abs(det(tform) - 1) <= epsilon)
        result = true;
    end
end
