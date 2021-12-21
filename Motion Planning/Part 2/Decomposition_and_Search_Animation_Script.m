% Decomposition and Search Method Animation

clc; clear; close all;

%% Workspace:
% Add Folders
addpath('Data')
addpath('Functions')

% Load Data
load Obstacles.mat
load Path.mat
load Roadmap.mat
load Trapezoids.mat

%% Initialize:
% Trapezoidation
T = trapezoids;

% Start and Finish Points
S = Start;
G = Goal;

% Path
dx = 20;
Path_x = Path(:, 1);
Path_y = Path(:, 2);

% Counter
i = 1;     % Construct Line
cc = 1;    % Color Index

%% Trapezoid Decomposition Gif:
% Gif File Name
filename1 = 'Decomposition_and_Search_Method_Decomposition.gif';

figure(1);  % Trapezoidation
axis([0, 500, 0, 500]);

hold on;
fill(P1(:, 1), P1(:, 2), 'k');  % Obstacles
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');

for ii = 1:length(T)
    a = rand; b = rand; c = rand;
    X = T{ii}(:, 1); Y = T{ii}(:, 2);  % Trapezoid Decomposition
    fill(X, Y, [a b c]);  
    
    % Animate Script
    drawnow limitrate
    frame = getframe(gca);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write Animation To .gif File
    if ii == 1
        imwrite(imind, cm, filename1, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename1, 'gif', 'DelayTime', 0.1, 'WriteMode', 'append');
    end
    
    % Count
    if mod(ii, 1000) == 0
        disp(num2str(ii));
    end    
end

hold off;

%% Motion Planning Gif
filename2 = 'Decomposition_and_Search_Method_Animated.gif';
figure(2);  % Motion Planning 
axis([0, 500, 0, 500]);  

hold on;
fill(P1(:, 1), P1(:, 2), 'k');  % Obstacles
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');

for iii = 1:length(Road)  
    for jj = 1:size(Road, 2)
        if isempty(Road{iii, jj}) ~= 1
            plot(Road{iii, jj}(:, 1), Road{iii, jj}(:, 2), ...  % Roadmap
                'Color', '#0072BD', 'linewidth', 1);
        end
    end
    % Animate Script
    drawnow limitrate
    frame = getframe(gca);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write Animation To .gif File
    if ii == 1
        imwrite(imind, cm, filename2, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename2, 'gif', 'DelayTime', 0.1, 'WriteMode', 'append');
    end
    
    % Count
    if mod(ii, 1000) == 0
        disp(num2str(ii));
    end
    
end

for ii = 1:length(V)  % Node Numbers
    x = V{ii}(1); y = V{ii}(2);
    plot(x, y, 'o');
    text(x, y - 1, sprintf('%d', ii));
    
    % Animate Script
    drawnow limitrate
    frame = getframe(gca);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write Animation To .gif File
    if ii == 1
        imwrite(imind, cm, filename2, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename2, 'gif', 'DelayTime', 0.1, 'WriteMode', 'append');
    end
    
    % Count
    if mod(ii, 1000) == 0
        disp(num2str(ii));
    end
    
end

for iii = 1:length(Road)
    for jj = 1:size(Road, 2)
        if isempty(Road{iii, jj}) ~= 1
            plot(Road{iii, jj}(:, 1), Road{iii, jj}(:, 2), ...  % Roadmap
                'Color', '#0072BD', 'linewidth', 1);
        end
    end
end

for iii = 1:length(V)  % Node Numbers
    x = V{iii}(1); y = V{iii}(2);
    plot(x, y, 'o');
    text(x, y - 1, sprintf('%d', iii));
end

plot(S(1), S(2), '*', 'linewidth', 3, 'Color', '#77AC30');  % Start and Goal Points
plot(G(1), G(2), '*', 'linewidth', 3, 'Color', '#FF0000');

fill(P1(:, 1), P1(:, 2), 'k');  % Obstacles
fill(P2(:, 1), P2(:, 2), 'k');
fill(P3(:, 1), P3(:, 2), 'k');
fill(P4(:, 1), P4(:, 2), 'k');

xlabel('x [m]'); ylabel('y [m]');

for ii = 1:dx:length(Path_x) % Path to Goal 
    
    hold on;
    plot(Path_x(i:dx:ii), Path_y(i:dx:ii), ...  
        '-', 'linewidth', 3, 'Color', '#D95319'); 
    hold off;
        
    % Animate Script
    drawnow limitrate
    frame = getframe(gca);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write Animation To .gif File
    if ii == 1
        imwrite(imind, cm, filename2, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename2, 'gif', 'DelayTime', 0.1, 'WriteMode', 'append');
    end
    
    % Count
    if mod(ii, 1000) == 0
        disp(num2str(ii));
    end
    
end