function result = isIdentity(A)
    result = false;
    if ( isdiag(A) && all(~(diag(A) - 1)) )
        result = true;
    end
end
