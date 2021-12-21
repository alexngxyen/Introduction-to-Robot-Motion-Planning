function [Start, Goal] = RandPosition()
%=========================================================================%
% RandPosition randomizes the start and finish points in the environment.
%
%  Outputs:    
%   Start  -  Randomized Start Point
%    Goal  -  Randomized Goal Point 
%=========================================================================%

% Obstacles
P1 = [104.0000,  179.0000;
    97.0000,  254.0000;
    139.0000,  263.0000;
    173.0000,  254.0000;
    178.0000,  190.0000;
    144.0000,  176.0000];

P2 = [171.0000,   83.0000;
  190.0000,  116.0000;
  222.0000,  137.0000;
  275.0000,   96.0000;
  259.0000,   63.0000;
  228.0000,   37.0000];

P3 = [  194.0000,  269.0000;
  147.0000,  330.0000;
  296.0000,  445.0000;
  342.0000,  383.0000;
  244.0000,  308.0000;
  334.0000,  189.0000;
  434.0000,  267.0000;
  474.0000,  208.0000;
  329.0000,   95.0000;
  283.0000,  153.0000;
  236.0000,  213.0000];

P4 = [  291.0000,  296.0000;
  320.0000,  333.0000;
  353.0000,  359.0000;
  391.0000,  336.0000;
  403.0000,  294.0000;
  374.0000,  268.0000;
  332.0000,  250.0000];


% Initialize Start
p1 = P1(1, :);

% Inpolygon Test
log1 = inpolygon(p1(1), p1(2), P1(:, 1), P1(:, 2));
log2 = inpolygon(p1(1), p1(2), P2(:, 1), P2(:, 2));
log3 = inpolygon(p1(1), p1(2), P3(:, 1), P3(:, 2));
log4 = inpolygon(p1(1), p1(2), P4(:, 1), P4(:, 2));

while log1 == 1 || log2 == 1 || log3 == 1 || log4 == 1
    % Randomize 
    p1 = round(rand(1, 2)*500);
    
    % Inpolygon Test
    log1 = inpolygon(p1(1), p1(2), P1(:, 1), P1(:, 2));
    log2 = inpolygon(p1(1), p1(2), P2(:, 1), P2(:, 2));
    log3 = inpolygon(p1(1), p1(2), P3(:, 1), P3(:, 2));
    log4 = inpolygon(p1(1), p1(2), P4(:, 1), P4(:, 2));
end

% Initialize Goal
p2 = p1;

% Inpolygon Test
log1 = inpolygon(p2(1), p2(2), P1(:, 1), P1(:, 2));
log2 = inpolygon(p2(1), p2(2), P2(:, 1), P2(:, 2));
log3 = inpolygon(p2(1), p2(2), P3(:, 1), P3(:, 2));
log4 = inpolygon(p2(1), p2(2), P4(:, 1), P4(:, 2));
Z = norm(p2 - p1);

while log1 == 1 || log2 == 1 || log3 == 1 || log4 == 1 ||  Z == 0 
    % Randomize 
    p2 = round(rand(1, 2)*500);
    
    % Inpolygon Test
    log1 = inpolygon(p2(1), p2(2), P1(:, 1), P1(:, 2));
    log2 = inpolygon(p2(1), p2(2), P2(:, 1), P2(:, 2));
    log3 = inpolygon(p2(1), p2(2), P3(:, 1), P3(:, 2));
    log4 = inpolygon(p2(1), p2(2), P4(:, 1), P4(:, 2)); 
    Z = norm(p2 - p1);
end

% Start and Goal Point
Start = p1;
Goal = p2;

end
