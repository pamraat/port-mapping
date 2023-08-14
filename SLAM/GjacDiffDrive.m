function G = GjacDiffDrive(x, u)
    eps = 1e-4*ones(size(x));
    for i=1:length(x)
        a = zeros(size(x)); a(i)= eps(i);
        G(:, i) = (gDiffDrive(x + a, u) - gDiffDrive(x, u))/eps(i);
    end
end