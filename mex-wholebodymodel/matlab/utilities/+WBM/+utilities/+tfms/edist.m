function ed = edist(v1, v2)
    % Calculates the Euclidean distance of two (large) vectors.
    v  = v1(:) - v2(:);
    ed = sqrt(v.'*v);
end
