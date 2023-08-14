function RunSLAM(SLAM)

%% Problem Setup
dt = SLAM(2:end, 1) - SLAM(1:end - 1, 1);
u = [dt SLAM(2:end, 2:3)];
z = SLAM(2:end, 4:end);
g = @(x, u) gDiffDrive(x, u);
G = @(x, u) GjacDiffDrive(x, u);
h = @(x) hSLAM(x);
H = @(x) HjacSLAM(x);
R = [0.01 0.001 0.002;
     0.001 0.01 0.002;
     0.002 0.002 0.015];
R(end + 1:end+size(z, 2), end + 1:end+size(z, 2)) = 0;
Q = diag(0.1*ones(size(z, 2), 1));

%% EKF
mu = [zeros(1, 3) SLAM(1, 4:end)];
sigma = R(1:3, 1:3); sigma(end+1:end+size(z, 2), end+1:end+size(z, 2)) = Q;

for i = 1:length(dt)
    [munew, sigmanew] = EKF(mu(end, :)', sigma(end - size(mu, 2) + 1:end, :), u(i, :), z(i, :)', R, Q, g, G, h, H);
    mu = [mu; munew'];
    sigma = [sigma; sigmanew];
end
obs = reshape(mu(end, 4:end)', 2, size(z, 2)/2)';
obs1 = reshape(mu(end-10, 4:end)', 2, size(z, 2)/2)';
obs2 = reshape(mu(end-20, 4:end)', 2, size(z, 2)/2)';

figure(1)
hold on
p1 = plot(mu(:, 1), mu(:, 2), 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'EKF-Robot Trajectory');
for i = 1:1:size(mu, 1)
    p2 = plotCovEllipse(mu(i, 1:2), sigma(size(mu, 2)*(i - 1) + [1 2], 1:2), 1, [{'color'},{'m'}, {'DisplayName'}, {'Trajectory 1-Sigma'}]);
end
hold on
for i = 1:size(z, 2)/2
    p3 = plot(mu(:, 2*(i + 1)), mu(:, 2*(i + 1) + 1), 'red',  'LineWidth', 0.5, 'LineStyle', '--', 'DisplayName', 'EKF-Marker');
end
scatter(obs(:, 1), obs(:, 2), 50, 'filled', 's', 'MarkerFaceColor', 'black', 'DisplayName', 'Final Marker');
for i = 1:2:size(obs, 1)
    p5 = plot(obs(i:i + 1, 1), obs(i:i + 1, 2), 'Linewidth', 1, 'Color', 'black', 'DisplayName', 'Final Wall');
end
scatter(obs1(:, 1), obs1(:, 2), 50, 'filled', 's', 'MarkerFaceColor', 'cyan', 'DisplayName', 'Final Marker');
for i = 1:2:size(obs1, 1)
    p7 = plot(obs1(i:i + 1, 1), obs1(i:i + 1, 2), 'Linewidth', 1, 'Color', 'cyan', 'DisplayName', 'Wall at 20s');
end
scatter(obs2(:, 1), obs2(:, 2), 50, 'filled', 's', 'MarkerFaceColor', 'green', 'DisplayName', 'Final Marker');
for i = 1:2:size(obs2, 1)
    p6 = plot(obs2(i:i + 1, 1), obs2(i:i + 1, 2), 'Linewidth', 1, 'Color', 'green', 'DisplayName', 'Wall at 10s');
end
legend([p1 p2 p3 p5 p6 p7],"Interpreter","tex", 'Location', 'best');
hold on
title("EKF SLAM - Robot Trajectory", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [200, 150, 600, 500]);
hold off

%% Fast PF SLAM

nParticles = 100;
X = [zeros(3, 1); SLAM(1, 4:end)']; X = repmat(X, 1, nParticles);
w = ones(1, nParticles);
sigma = R(1:3, 1:3); sigma(end+1:end+size(z, 2), end+1:end+size(z, 2)) = Q;
sigma = repmat(sigma, 1, 1, nParticles);

for i = 1:length(dt)
    [Xnew, wnew, sigmanew] = fastSLAM(X(end - size(z, 2) - 2:end, :), sigma(end - size(z, 2) - 2:end, :, :), u(i, :), z(i, :)', R, Q, g, G, h, H);
    X = [X; Xnew];
    w = [w; wnew];
    sigma = [sigma; sigmanew];
end
[~, maxind] = max(w, [], 2); idx = (1:3+size(z, 2):size(X, 1))';
obs = reshape(X(end - size(z, 2) + 1:end, maxind(end)), 2, size(z, 2)/2)';

figure(2)
hold on
p1 = plot(X(sub2ind(size(X), idx, maxind)), X(sub2ind(size(X), idx + 1, maxind)), 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'PF-Robot Trajectory');
plot(X(sub2ind(size(X), idx + 3, maxind)), X(sub2ind(size(X), idx + 4, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
plot(X(sub2ind(size(X), idx + 5, maxind)), X(sub2ind(size(X), idx + 6, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
plot(X(sub2ind(size(X), idx + 7, maxind)), X(sub2ind(size(X), idx + 8, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
plot(X(sub2ind(size(X), idx + 9, maxind)), X(sub2ind(size(X), idx + 10, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
plot(X(sub2ind(size(X), idx + 11, maxind)), X(sub2ind(size(X), idx + 12, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
plot(X(sub2ind(size(X), idx + 13, maxind)), X(sub2ind(size(X), idx + 14, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--');
p3 = plot(X(sub2ind(size(X), idx + 15, maxind)), X(sub2ind(size(X), idx + 16, maxind)), 'red',  'LineWidth', 0.5, 'LineStyle', '--', 'DisplayName', 'EKF-Marker');
p4 = scatter(obs(:, 1), obs(:, 2), 50, 'filled', 's', 'MarkerFaceColor', 'black', 'DisplayName', 'Marker');
for i = 1:2:size(obs, 1)
    p5 = plot(obs(i:i + 1, 1), obs(i:i + 1, 2), 'Linewidth', 1, 'Color', 'black', 'DisplayName', 'Wall');
end
legend([p1 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
hold on
title("Fast SLAM - Robot Trajectory", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [600, 150, 600, 500]);
hold off