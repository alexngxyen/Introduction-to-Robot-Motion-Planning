% MAE 195 Introduction to Robot Motion Planning and Navigation
% Project 1
% Description: Create workspace shown in picture (three chairs and a desk).
% Date: 4/20/21
% Author: Alex Nguyen

clc; clear; close all;

%% Load:
% Folders
addpath('Functions')
addpath('Data')

% Data
load('Obstacles.mat')

%% Import Image & Create Obstacles:
Create = 0;  % (1) Create Obstacles; (0) Do Not Create Obstacles

if Create == 1
    % Load & Show Image
    imshow(imread('workspace.jpg'));
    
    % Get Data Points
    h = drawpolygon;
end

%% Plot Workspace:
% Start & Finish Points
Start = [200, 25];
Finish = [275, 300];

% Line Through Start and Finish Points
[a, b, c] = computeLineThroughTwoPoints(Start, Finish);
m = -a/b;
yint = -c/b;
x = Start(1):1:Finish(1);

% Obstacles
figure(1); 
hold on;
fill(P1(:, 1), P1(:, 2), 'k');
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');
hold off;

% Start & End Points
figure(1);
hold on;
plot(Start(1), Start(2), '*', 'linewidth', 3, 'Color', '#77AC30');
plot(Finish(1), Finish(2), '*', 'linewidth', 3, 'Color', '#77AC30');
plot(x, m*x + yint, 'r--', 'linewidth', 2);
title('Approximate Obstacles in Workspace');
xlabel('x [m]'); ylabel('y [m]');
hold off;


