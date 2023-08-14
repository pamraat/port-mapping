clear

load dataStore.mat

n = 50;
m = 50;

x = linspace(-2.5, 2.5, n + 1);
y = linspace(-2.5, 2.5, m + 1);
xC = linspace(x(1), x(end - 1), n) + (x(end) - x(1))/(2*n);
yC = linspace(y(1), y(end - 1), m) + (y(end) - y(1))/(2*m);
[X, Y] = meshgrid(x, y);
[XC, YC] = meshgrid(xC, yC);
lt = zeros(size(XC));

%% Bump Sensor
% Q = [0.005 0; 0 0.005];
% robRad = 0.16;
% 
% for i=1:size(dataStore.bump, 1)
%     lt = logOddsBump(dataStore.truthPose(i, 2:end), lt, dataStore.bump(i, 2:end), Q, xC, yC, robRad);
%     pdf = exp(lt)./(1 + exp(lt));
% end
% 
% pdf(pdf>0.5) = 1;
% pdf(pdf<1) = 0;
% %% Plot
% load 'loopMap.mat'
% map = loopMap;
% C = [0.6 0.6 0.6];
% plotGridBelief(xC, yC, pdf)
% hold on
% p1 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'O/H localization');
% [ind1, ~] = find(dataStore.bump == 1);
% p2 = scatter(dataStore.truthPose(ind1, 2), dataStore.truthPose(ind1, 3), '*', 'red', 'DisplayName', 'Bump Location');
% plot(X, Y, 'Color', C)
% plot(X', Y', 'Color', C)
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([p1 p2],"Interpreter","tex", 'Location', 'best');
% hold on
% title("Robot Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% xlim([-2.7 2.7])
% ylim([-2.7 2.7])
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% Depth Sensor
Q = 0.01*eye(2);
sensorPos = [0 0.08];
angles = linspace(27, -27, 9)'*pi/180;

vid = VideoWriter("video1");
open(vid);

%% Plot
figure(1)
load 'loopMap.mat'
map = loopMap;
C = [0.6 0.6 0.6];

for i=1:100:size(dataStore.rsdepth, 1)
    hold on
    lt = logOddsDepth(dataStore.truthPose(i, 2:end), lt, dataStore.rsdepth(i, 3:end)', Q, xC, yC, sensorPos, angles);
    lt(lt>300) = 300;
    pdf = exp(lt)./(1 + exp(lt));
    plotGridBelief(xC, yC, pdf)
    p1 = scatter(dataStore.truthPose(i, 2), dataStore.truthPose(i, 3)', 100, 'blue', 'LineWidth', 1, 'DisplayName', 'O/H localization');
    quiver(dataStore.truthPose(i, 2), dataStore.truthPose(i, 3)', 0.2*cos(dataStore.truthPose(i, 4)), 0.2*sin(dataStore.truthPose(i, 4)), 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'blue', 'AutoScale','off');
    plot(X, Y, 'Color', C)
    plot(X', Y', 'Color', C)
    for i = 1:size(map, 1)
        plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
    end
    legend(p1,"Interpreter","tex", 'Location', 'best');
    title("Robot Trajectory", "Interpreter","tex");
    xlabel("X (m)");
    ylabel("Y (m)");
    xlim([-2.7 2.7])
    ylim([-2.7 2.7])
    fontsize(gca,14,"points");
    set(gcf, 'Position',  [400, 150, 600, 500]);
    frame = getframe(gcf);
    writeVideo(vid, frame);
    clf
end
close(vid)