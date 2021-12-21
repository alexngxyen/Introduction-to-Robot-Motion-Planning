% Bug 1 Animation Script

clc; clear; close all;

%% Workspace:
% Add Folders
addpath('Data')
addpath('Functions')

% Load Data
load('Obstacles.mat')
load('Path.mat');

%% Initialize:
% Start & Finish Points
Start = start;
Finish = goal;

% Path
dx = 30;
x = path(:, 1);
y = path(:, 2);

% Color
C = {'#4DBEEE'};

% Counter
i = 1;     % Construct Line
cc = 1;    % Color Index

%% Animation .gif File:
% Initialize Figure
figure(1)
xlabel('x [m]'); ylabel('y [m]');

% Gif File Name
filename = 'Bug1_Randomized.gif';

for ii = 1:dx:length(x)
  
    hold on;
    % Start and Goal Points
    plot(Start(1), Start(2), '*', 'linewidth', 3, 'Color', '#77AC30');
    plot(Finish(1), Finish(2), '*', 'linewidth', 3, 'Color', '#FF0000');
    
    % Obstacles
    fill(P1(:, 1), P1(:, 2), 'k');
    fill(P2(:, 1), P2(:, 2), 'k');
    fill(P3(:, 1), P3(:, 2), 'k');
    fill(P4(:, 1), P4(:, 2), 'k');
       
    % Path to Goal
    plot(x(i:dx:ii), y(i:dx:ii), '-', 'linewidth', 3, 'Color', C{1});
    hold off;
    
    if sum(x(i:ii) == x(ii)) == 2 && sum(y(i:ii) == y(ii)) == 2
        % Update Counter
        i = ii;
        
    end

    % Animate Script
    drawnow limitrate
    frame = getframe(gca);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write Animation To .gif File
    if ii == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);   
    else
        imwrite(imind, cm, filename, 'gif', 'DelayTime', 0.1, 'WriteMode', 'append');
    end
    
    % Count
    if mod(ii, 1000) == 0
        disp(num2str(ii));
    end
    
end
