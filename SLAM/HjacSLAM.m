function H = HjacSLAM(x)
    eps = 1e-4*ones(size(x));
    for i=1:length(x)
        a = zeros(size(x)); a(i)= eps(i);
        H(:, i) = (hSLAM(x + a) - hSLAM(x))/eps(i);
    end
end