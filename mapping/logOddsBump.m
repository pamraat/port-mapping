function lt = logOddsBump(pose, lt_1, z, Q, xC, yC, robRad)
    pos = robot2global(pose, [0 -robRad*z(1); 0 robRad*z(2); robRad*z(6) 0]);
    XGrid = repmat(xC, 1, length(yC))'; YGrid = repelem(yC, 1, length(xC))';
    if all(z == 0)
        w = mvnpdf([XGrid, YGrid], pose(1:2), Q);
        w = reshape(sum(w, 2)/sum(w, 'all'), length(xC), length(yC))';
        lt = lt_1 + log((1 - w)./(1 + w));
        return;
    end
    for i = 1:3
        if z(factorial(i)) == 0 w(:, i) = zeros(size(XGrid)); continue; end 
        w(:, i) = mvnpdf([XGrid, YGrid], pos(i, :), Q);
    end
    w = reshape(sum(w, 2)/sum(w, 'all'), length(xC), length(yC))';
    lt = lt_1 + log((1 + w)./(1 - w));
end