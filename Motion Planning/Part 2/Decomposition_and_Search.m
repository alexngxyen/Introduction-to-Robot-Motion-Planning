% Decomposition and Search Method

clc; clear; close all;

% Add Folders to Path
addpath('Functions');
addpath('Data');


% Load Obstacles
load('Obstacles.mat')
P1 = [P1(:, 1) P1(:, 2)];
P2 = [P2(:, 1) P2(:, 2)];
P3 = [P3(:, 1) P3(:, 2)];
P4 = [P4(:, 1) P4(:, 2)];
Polygon = {P1, P2, P3, P4};


% Sweeping Trapezoid Algorithm
[trapezoids, segments] = SweepingTrapezoidation(Polygon);


% Roadmap from Decomposition Algorithm
[V, Road, AdjTable, S] = RoadmapFromDecomposition(trapezoids, segments);


% Initialize Start and Goal Point
[Start, Goal] = RandPosition();
[Path_S, ind_S] = NearestNode(V, Start, Polygon);
[Path_G, ind_G] = NearestNode(V, Goal, Polygon);


% Planning via Decomposition and Search
[pVec, pNodes] = computeBFStree(AdjTable, ind_S);
NodePath = computeBFSpath(AdjTable, ind_S, ind_G);


% Motion Planning Path
Path = ShortestEdgePath(V, NodePath, Path_S, Path_G);


% Plot 
figure;
hold on

plot([0, 500, 500, 0, 0], [0, 0, 500, 500, 0], 'k', 'linewidth', 3);  % Workspace Boundaries

fill(P1(:, 1), P1(:, 2), 'k');  % Obstacles
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');
xlim([0 500]);
ylim([0 500]);

plot(Start(1), Start(2), 'go', 'linewidth', 4);  % Start and Goal Points
text(Start(1), Start(2), 'Start');
plot(Goal(1), Goal(2), 'ro', 'linewidth', 4);
text(Goal(1), Goal(2), 'Goal');

plot(S.T(:, 1), S.T(:, 2), 'd');  % Trapezoid and Line Segment Centroids
plot(S.L(:, 1), S.L(:, 2), 'o');

for ii = 1:length(Road)
    for jj = 1:size(Road, 2)
        if isempty(Road{ii, jj}) ~= 1
            plot(Road{ii, jj}(:, 1), Road{ii, jj}(:, 2), ...  % Roadmap
                'Color', '#0072BD', 'linewidth', 1);
        end
    end
end

for ii = 1:length(V)  % Node Numbers 
    x = V{ii}(1); y = V{ii}(2);
    plot(x, y, 'o');
    text(x, y - 1, sprintf('%d', ii));
end

plot(Path(:, 1), Path(:, 2), 'Color', '#D95319', 'linewidth', 2.5); % Decomposition and Search Strategy

hold off;
axis([0, 500, 0, 500]);
grid on;


% Save Data 
save('Path.mat', 'Path', 'NodePath');
save('Trapezoids.mat', 'trapezoids', 'segments');
save('Roadmap.mat', 'Start', 'Goal', 'Road', 'V', 'AdjTable');

