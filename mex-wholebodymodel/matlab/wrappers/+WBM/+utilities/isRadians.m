function result = isRadians(x)
    result = 0;

    if ~isa(x, 'double')
        return
    end

    % ignore NaN ...
    x = x(~isnan(x));
    if isempty(x)
        return
    end

    if ~isscalar(x)
        % find the min. and the max. value of the array ...
        minv = min(x);
        maxv = max(x);
    else % x is a single value ...
        minv = x;
        maxv = minv;
    end

    % range test ...
    if ( (minv >= -pi) && (maxv <= pi) )
        result = 1;
        return
    end
    if ( (minv >= 0) && (maxv <= 2*pi) )
        result = 2;
    end
end
