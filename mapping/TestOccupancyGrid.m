function [lFinalBump,lFinalDepth]=TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% TESTOCCUPANCYGRID 
% Returns and plots the final occupancy grids using the bump and depth sensors.
% Will create two (2) figures containing the occupancy grids.
%
%       TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
%
%       INPUTS:
%           dataStore   struct from running SimulatorGUI
%           l_0         initial log odds (scalar value)
%           NumCellsX   number of cells in the x direction (integer)
%           NumCellsY   number of cells in the y direction (integer)
%           boundaryX   boundary of the environment in the x direction.
%                       1 x 2 array [min_x max_x]
%           boundaryY   boundary of the environment in the Y direction.
%                       1 x 2 array [min_y max_y]
%       OUTPUTS:
%           lFinalBump  Final occupancy grid using bump sensor
%           lFinalDepth Final occupancy grid using depth sensor
%       Figures Created:
%           Figure 1    Occupancy grid using bump sensor
%           Figure 2    Occupancy grid from depth information

% Cornell University
% Autonomous Mobile Robots
% Homework #5
% Last, First Name

    n = NumCellsX;
    m = NumCellsY;
    
    x = linspace(boundaryX(1), boundaryX(2), n + 1);
    y = linspace(boundaryY(1), boundaryY(2), m + 1);
    xC = linspace(x(1), x(end - 1), n) + (x(end) - x(1))/(2*n);
    yC = linspace(y(1), y(end - 1), m) + (y(end) - y(1))/(2*m);
    [X, Y] = meshgrid(x, y);
    lt = l_0;
    
%% Bump Sensor
    Q = [0.005 0; 0 0.005];
    robRad = 0.16;
    
    for i=1:size(dataStore.bump, 1)
        lt = logOddsBump(dataStore.truthPose(i, 2:end), lt, dataStore.bump(i, 2:end), Q, xC, yC, robRad);
        pdf = exp(lt)./(1 + exp(lt));
    end
    lFinalBump = pdf;
%% Bump Plot
    C = [0.6 0.6 0.6];
    plotGridBelief(xC, yC, pdf)
    hold on
    p1 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'O/H localization');
    [ind1, ~] = find(dataStore.bump == 1);
    p2 = scatter(dataStore.truthPose(ind1, 2), dataStore.truthPose(ind1, 3), '*', 'red', 'DisplayName', 'Bump Location');
    plot(X, Y, 'Color', C)
    plot(X', Y', 'Color', C)
    legend([p1 p2],"Interpreter","tex", 'Location', 'best');
    hold on
    title("Robot Trajectory", "Interpreter","tex");
    xlabel("X (m)");
    ylabel("Y (m)");
    xlim([-2.7 2.7])
    ylim([-2.7 2.7])
    fontsize(gca,14,"points");
    set(gcf, 'Position',  [400, 150, 600, 500]);
    
    %% Depth Sensor
    Q = 0.01*eye(2);
    sensorPos = [0 0.08];
    angles = linspace(27, -27, 9)'*pi/180;
    
    for i=1:size(dataStore.rsdepth, 1)
        lt = logOddsDepth(dataStore.truthPose(i, 2:end), lt, dataStore.rsdepth(i, 3:end)', Q, xC, yC, sensorPos, angles);
        lt(lt>300) = 300;
        pdf = exp(lt)./(1 + exp(lt));
    end
    lFinalDepth = pdf;
    %% Plot
    C = [0.6 0.6 0.6];
    plotGridBelief(xC, yC, pdf)
    hold on
    p1 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3)', 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'O/H localization');
    [ind1, ~] = find(dataStore.bump == 1);
    p2 = scatter(dataStore.truthPose(ind1, 2), dataStore.truthPose(ind1, 3), '*', 'red', 'DisplayName', 'Bump Location');
    plot(X, Y, 'Color', C)
    plot(X', Y', 'Color', C)
    legend([p1 p2],"Interpreter","tex", 'Location', 'best');
    hold on
    title("Robot Trajectory", "Interpreter","tex");
    xlabel("X (m)");
    ylabel("Y (m)");
    xlim([-2.7 2.7])
    ylim([-2.7 2.7])
    fontsize(gca,14,"points");
    set(gcf, 'Position',  [400, 150, 600, 500]);

end