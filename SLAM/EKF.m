function [munew, sigmanew] = EKF(mu, sigma, u, z, R, Q, g, G, h, H)
    muPred = g(mu, u);
    GJac = G(mu, u);
    if all(u == 0), R = zeros(size(R)); end
    sigmaPred = GJac*sigma*GJac' + R;
    zPred = h(muPred); zPred(z == 0) = 0;
    HJac = H(muPred);
    K = sigmaPred*HJac'/(HJac*sigmaPred*HJac' + Q);
    munew = muPred + K*(z - zPred);
    sigmanew = (eye(size(sigma, 1)) - K*HJac)*sigmaPred;
end