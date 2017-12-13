function result = isNormalized(v, epsilon)
    if (nargin ~= 2)
        epsilon = 1e-12; % min. value to treat a number as zero ...
    end
    v = v(:); % make sure that v is a column vector ...

    result = false;
    if ((v.'*v - 1) <= epsilon) % the .' is crucial to avoid computing the conjugate!
        % the vector is already normalized ...
        result = true;
    end
end
