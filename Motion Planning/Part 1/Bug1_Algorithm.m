% Bug 1 Algorithm

clc; clear; close all;

% Add Folders to Path
addpath('Functions')
addpath('Data')

% Load Obstacles
load('Obstacles.mat');

% Randomize Start and Goal Points in the Free Workspace 
[start, goal] = randPosition();

P1 = flip(P1);
P2 = flip(P2);
P3 = flip(P3);
P4 = flip(P4);
obstaclesList = {P1,P2,P3,P4};

% Step Size
step_size = 0.1;

% Compute Path
path = Bug1(start, goal, obstaclesList, step_size);

% Plot
figure;
hold on;
plot(start(1), start(2), '*', 'linewidth', 3, 'Color', '#77AC30');  % Start Point
plot(goal(1), goal(2), '*', 'linewidth', 3, 'Color', '#77AC30');    % End Point

fill(P1(:, 1), P1(:, 2), 'k');  % Obstacles
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');

plot(path(:, 1), path(:, 2), '-r', 'linewidth', 3);  % Path to Goal
hold off;

% Save Path
save('path.mat', 'path', 'start', 'goal');

