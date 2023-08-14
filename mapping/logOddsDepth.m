function lt = logOddsDepth(pose, lt_1, z, Q, xC, yC, sensorPos, angles)
    z(z==0) = 0.16;
    zCorr = robot2global([sensorPos 0], [z z.*tan(angles)]);
    pos = robot2global(pose, zCorr);
    XGrid = repmat(xC, 1, length(yC))'; YGrid = repelem(yC, 1, length(xC))';
    u1 = [-sin(angles(1)); cos(angles(1))]; u2 = [sin(angles(end)); -cos(angles(end))];
    Xp = global2robot(pose, [XGrid YGrid]);
    ind = find(sign(Xp*u1) < 0 & sign(Xp*u2) < 0);
    w1 = zeros(size(XGrid));
    for i = 1:length(ind)
        for j = 1:size(zCorr, 1)
            if j == size(zCorr, 1)
                if Xp(ind(i), 1) < zCorr(j, 1) && Xp(ind(i), 2) == zCorr(j, 2)
                    w1(ind(i)) = 1;
                end
                continue;
            end
            if Xp(ind(i), 1) < zCorr(j, 1) && Xp(ind(i), 2) <= zCorr(j, 2) && Xp(ind(i), 2) > zCorr(j + 1, 2)
                w1(ind(i)) = 1;
            end
        end
    end
    for i = 1:size(pos, 1)
        w(:, i) = mvnpdf([XGrid, YGrid], pos(i, :), Q);
    end
    w = reshape(sum(w, 2)/sum(w, 'all'), length(xC), length(yC))';
    w1 = reshape(sum(w1, 2)/sum(w1, 'all'), length(xC), length(yC))';
    if any(isnan(w1))
        w1 = zeros(size(w1));
    end
    lt = lt_1 + log((1 + w)./(1 - w)) + 10*log((1 - w1)./(1 + w1));
end