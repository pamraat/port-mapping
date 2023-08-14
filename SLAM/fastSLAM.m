function [Xnew, w, sigmanew] = fastSLAM(X, sigma, u, z, R, Q, g, G, h, H)
    if all(u == 0), R = zeros(size(R)); end
    for i = 1:size(X, 2)
        muPred = g(X(:, i), u);
        XPred(:, i) = muPred + mvnrnd(zeros(length(muPred), 1), R)';
        zPred(:, i) = h(XPred(:, i)); zPred(z == 0, i) = 0;
        w(1, i) = mvnpdf(zPred(:, i), z, Q);
        GJac = G(X(:, i), u);
        sigmaPred = GJac*sigma(:, :, i)*GJac';
        HJac = H(X(:, i));
        K = sigmaPred*HJac'/(HJac*sigmaPred*HJac' + Q);
        munew = muPred + K*(z - zPred(:, i));
        XPred(4:end, i) = munew(4:end);
        sigmanew(:, :, i) = (eye(size(sigma, 1)) - K*HJac)*sigmaPred;
    end
    if all(w == 0, 'all'), Xnew = XPred; return; end
    w = w/sum(w);
    idx = randsample(size(X, 2), size(X, 2), true, w);
    Xnew = XPred(:, idx); w = w(idx); sigmanew(:, :, idx);
end