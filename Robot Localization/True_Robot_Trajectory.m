% MAE 195 Introduction to Robot Motion Planning and Navigation
% Robot Localization Project
% Description: Beacon-Based Localization True Robot Trajectory
% Date: 6/4/21
% Author: Alex Nguyen

clc; clear; close all;

%% Initialize:

% For Loop-Closure Study; Pre-Defined Noise Samples
load w_x.mat;  
load w_y.mat;
load w_theta.mat;

% Simulation Parameters
T = 0.1;                 % Step Size
t_f = 307;               % Final Time
SimL = t_f/T;            % Number of Steps

% Measurement Zone r = {5, 10, 15}
m_r = 10; 

%% Landmark Locations:

% Preallocate
nl = 35;           % Number of Landmarks
Xl = cell(nl, 1);

% Grouped Landmarks by their radii
Xl{1} = [2; 3];
Xl{2} = [-2; -21];
Xl{3} = [6; -3.5];
Xl{4} = [3; -8.5];

Xl{5} = [17; 3];
Xl{6} = [18; -5];
Xl{7} = [52; -18];
Xl{8} = [25; -3.5];

Xl{9} = [32; 14];
Xl{10} = [38; 1];
Xl{11} = [36; 9.5];
Xl{12} = [30; 4];

Xl{13} = [52; 24];
Xl{14} = [53; 14];
Xl{15} = [63; 21.5];
Xl{16} = [62; 13];

Xl{17} = [75; 22];
Xl{18} = [70; 15];
Xl{19} = [82; 21.5];
Xl{20} = [78; 9];

Xl{21} = [42; 35];
Xl{22} = [32; 38];
Xl{23} = [47; 27];
Xl{24} = [35; 20];

Xl{25} = [45; 40.5];
Xl{26} = [40; 50];

Xl{27} = [60; 2];
Xl{28} = [56; -6];

Xl{29} = [48; -7];
Xl{30} = [35; -12];

Xl{31} = [25; -15];
Xl{32} = [20; -19];
Xl{33} = [10; -14];
Xl{34} = [5; -16];

Xl{35} = [63; -11];

% Visualize Landmark Locations
figure;
hold on;
for ii = 1:nl
    plot(Xl{ii}(1), Xl{ii}(2), 'o', 'markersize', 5);
end
hold off;
xlabel('x'); ylabel('y');

%% True Robot Motion:

% Initial States
x = [0, 0, 0.01]';

% Preallocate
Time = zeros(1, SimL);
u = zeros(2, SimL);
landmark_detection_indicator = cell(nl, 1);
j = 1;

for k = 1:SimL
    % Inputs Driving the True System
    Time(k + 1) = Time(k) + T;
    
    if Time(k) <= 20
        u(1, k) = 1;
        u(2, k) = 0;
        
    elseif (Time(k) > 20 && Time(k) <= 25)
        u(1, k) = 1;
        u(2, k) = 0.15;
    elseif (Time(k) > 25 && Time(k) <= 45)
        u(1, k) = 1;
        u(2, k) = 0;
        
    elseif (Time(k) > 45 && Time(k) <= 50)
        u(1, k) = 1;
        u(2, k) = -0.15;
        
    elseif (Time(k) > 50 && Time(k) <= 75)
        u(1, k) = 1;
        u(2, k) = 0;
        
    elseif (Time(k) > 75 && Time(k) <= 90)
        u(1, k) = 1;
        u(2, k) = -0.15;
        
    elseif (Time(k) > 90 && Time(k) <= 115)
        u(1, k) = 1;
        u(2, k) = 0;
        
    elseif (Time(k) > 115 && Time(k) <= 130)
        u(1, k) = 1.2;
        u(2, k) = -0.15;
        
    elseif (Time(k) > 130 && Time(k) <= 160)
        u(1, k) = 1.5;
        u(2, k) = 0;
        
    elseif (Time(k) > 160 && Time(k) <= 190)
        u(1, k) = 1.2;
        u(2, k) = -0.15;
        
    elseif (Time(k) > 190 && Time(k) <= 205)
        u(1, k) = 1.2;
        u(2, k) = 0.2;
        
    elseif (Time(k) > 205 && Time(k) <= 220)
        u(1, k) = 1.5;
        u(2, k) = 0;
        
    elseif (Time(k) > 220 && Time(k) <= 235)
        u(1, k) = 1.5;
        u(2, k) = -0.2;
        
    elseif (Time(k) > 235 && Time(k) <= 285)
        u(1, k) = 1.5;
        u(2, k) = 0;
        
    elseif (Time(k) > 285 && Time(k) <= 290)
        u(1, k) = 1.5;
        u(2, k) = -0.4;
        
    else
        u(1, k) = 1;
        u(2, k) = 0;
        
    end
    
    % True Robot Model
    x(1, k + 1) = x(1, k) + T*u(1, k)*cos(x(3, k)) + w_x(k);
    x(2, k + 1) = x(2, k) + T*u(1, k)*sin(x(3, k)) + w_y(k);
    x(3, k + 1) = x(3, k) + T*u(2, k) + w_theta(k);
    
    % Detect Landmarks in the Measurment Zone
    for i_l = 1:nl
        range_to_landmark_i = norm(x(1:2, k+1) - Xl{i_l});
        
        if range_to_landmark_i <= m_r
            landmark_detection_indicator{i_l}(k + 1) = 1;
            
        else
            landmark_detection_indicator{i_l}(k + 1) = 0;
        
        end
        
    end
end

%% Plot:

% True Robot Trajectory + Landmarks
figure;
hold on;
plot(x(1, :), x(2, :), 'r', 'LineWidth', 3);
for i = 1:nl
    % i-th Landmark
    plot(Xl{i}(1), Xl{i}(2), 'k*', 'MarkerSize', 12);
end
plot(x(1, 1), x(2, 1), 'x', 'MarkerSize', 12); % Robot Start Point
legend('Robot Trajectory', 'Landmarks', 'location', 'best');
axis([-20 100 -40 80]);
xlabel('x'); ylabel('y');
grid on;

% Visualizing the Measurement Zone at Various Locations Along the Path
radii = [5, 10, 15]'; % Three Possible Measurement Ranges

for k = 1:750:SimL
    centers = [x(1, k), x(2, k) ; ...
               x(1, k), x(2, k) ; ...
               x(1, k), x(2, k)];
           
    viscircles(centers, radii, 'Color', randi([1, 100], 1, 3)/100);
end
hold off;
set(gcf, 'Position', [0, 0, 1000, 1000]); % Size of the Figure Window
pbaspect([1, 1, 1]);                      % Aspect Ratio of the Plot
set(findobj(gcf, 'type', 'axes'), ...
    'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'Bold', 'LineWidth', 2);


% Detected Landmarks at Each Time
figure;
hold on;
for i_l = 1:nl
    plot(Time(2:SimL + 1), landmark_detection_indicator{i_l}(2:SimL + 1)*i_l);
end
xlabel('Time (s)'); ylabel('Landmarks');
yticks([1:nl]);
grid on;
set(findobj(gcf, 'type', 'axes'), ...
    'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'Bold', 'LineWidth', 2)

%% Save Data:
% True Robot Trajectory
% save('TrueRobotTraj.mat', 'x', 'Xl');