% MAE 195 Introduction to Robot Motion Planning and Navigation
% Robot Localization Project
% Description: Beacon-Based Localization Estimated Trajectories for each
% case.
% Date: 6/5/21
% Author: Alex Nguyen

clc; clear; close all;

%% Load Data:

% Add Path
addpath Data

% True Data
load TrueRobotTraj.mat

% Estimated Data
load Radii5.mat
load Radii10.mat
load Radii15.mat

%% Trajectory Plot:
nl = length(Xl);

figure;
hold on;
% True Trajectory
plot(x(1, :), x(2, :), 'r', 'LineWidth', 2);

% Estimated Trajectories
plot(xhat5(1, :), xhat5(2, :), 'Color', '#4DBEEE', 'LineWidth', 2);
plot(xhat10(1, :), xhat10(2, :), 'Color', '#7E2F8E', 'LineWidth', 2);
plot(xhat15(1, :), xhat15(2, :), 'Color', '#77AC30', 'LineWidth', 2);

for i = 1:nl
    % i-th Landmark
    plot(Xl{i}(1), Xl{i}(2), 'k*', 'MarkerSize', 12);
end
plot(x(1, 1), x(2, 1), 'x', 'MarkerSize', 12); % Robot Start Point
hold off;
legend('True Robot Trajectory', 'Estimated Robot Trajectory (Radii 5 m)', 'Estimated Robot Trajectory (Radii 10 m)', ...
    'Estimated Robot Trajectory (Radii 15 m)', 'True Landmarks', 'location', 'best');
axis([-20 100 -40 80]);
xlabel('x'); ylabel('y');
grid on;

set(gcf, 'Position', [0, 0, 1000, 1000]); % Size of the Figure Window
pbaspect([1, 1, 1]);                      % Aspect Ratio of the Plot
set(findobj(gcf, 'type', 'axes'), ...
    'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'Bold', 'LineWidth', 2);

%% Performance:
fprintf('Measurement radii of 5: \n');
fprintf('\t\tLoop-closure Distance = %4.4f \n', loop_closure_dist5);
fprintf('\t\tLoop-closure Error = [%4.4f, %4.4f]^T \n\n', loop_closure5(1:2));

fprintf('Measurement radii of 10: \n');
fprintf('\t\tLoop-closure Distance = %4.4f \n', loop_closure_dist10);
fprintf('\t\tLoop-closure Error = [%4.4f, %4.4f]^T \n\n', loop_closure10(1:2));

fprintf('Measurement radii of 15: \n');
fprintf('\t\tLoop-closure Distance = %4.4f \n', loop_closure_dist15);
fprintf('\t\tLoop-closure Error = [%4.4f, %4.4f]^T \n\n', loop_closure15(1:2));

