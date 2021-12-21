% MAE 195 Introduction to Robot Motion Planning and Navigation
% Robot Localization Project
% Description: SLAM-Based Localization Monte-Carlo RMSE.
% Date: 6/14/21
% Author: Alex Nguyen

clc; clear; close all;

%% Initialize:

% Add Data Folder to Path
addpath Data;

% For Loop-Closure Study; Pre-Defined Noise Samples
load w_x.mat;
load w_y.mat;
load w_theta.mat;

% Simulation Parameters
T = 0.1;                 % Step Size
t_f = 307;               % Final Time
SimL = t_f/T;            % Number of Steps
nx = 73;

% Measurement Equations
h = @(r, rbi) norm(r(1:2) - rbi);
phi = @(r, rbi) atan2(r(2) - rbi(2), r(1) - rbi(1)) - r(3);

% Linearized State Transition Matrix
Fk = @(x, u) [1, 0, -sin(x(3))*u(1)*T  ; ...
    0, 1,  cos(x(3))*u(1)*T  ; ...
    0, 0,                 1 ];

% Linearized Observation Jacobian
R1 = @(r, rbi) sqrt((r(1) - rbi(1))^2 + (r(2) - rbi(2))^2);
R2 =  @(r, rbi) 1/(1 + ((r(2) - rbi(2))/(r(1) - rbi(1)))^2);
Hv = @(r, rbi)  [                 (r(1) - rbi(1))/R1(r, rbi),   (r(2) - rbi(2))/R1(r, rbi),  0 ; ...  % Vehicle
    -R2(r, rbi)*(r(2) - rbi(2))/(r(1) - rbi(1))^2, R2(r, rbi)/(r(1) - rbi(1))^2, -1];

Hl = @(r, rbi)  [                 -(r(1) - rbi(1))/R1(r, rbi),   -(r(2) - rbi(2))/R1(r, rbi) ; ...    % Landmark(s)
    R2(r, rbi)*(r(2) - rbi(2))/(r(1) - rbi(1))^2, -R2(r, rbi)/(r(1) - rbi(1))^2];

% Noise
Qpv = diag([(0.4*T)^2, (0.4*T)^2, (0.05*T)^2]);
Ri = diag([(0.1)^2, (3*pi/180)^2]);
r = sqrtm(Ri);

% Estimation Error Covariance
Ppv = diag([1, 1, 0.1]);

% Measurement Zone r = {5, 10, 15}
m_r = 15;

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

%% Initialize Matrices & Landmarks States:

% Preallocate
xl_true = [];
xl_hat = [];
Pl = cell(nl, 1);
Fi = Pl;
Qi = Fi;

for ii = 1:nl
    % State Transition Matrix
    Fi{ii} = eye(2);
    
    % Process Noise Covariance Matrix
    Qi{ii} = eps*eye(2);
    
    % Estimation Error Covariance
    Pl{ii} = eye(2);
    
    % True Landmark States
    xl_true = [xl_true, Xl{ii}'];
    
    % Estimate Landmark States
    rng(ii);
    xl_hat = [xl_hat, mvnrnd(Xl{ii}, Pl{ii})];
   
end

%% Beacon-Based Localization Monte Carlo:

% Monte Carlo Parameters
MC_run = 50;          % Number of Monte Carlo Runs
noise_channels = nl;
[indep_noise{1:noise_channels*MC_run}] = ...
    RandStream.create('mrg32k3a', 'NumStreams', noise_channels*MC_run);

% Preallocate
z_noise = cell(nl, MC_run);
x_noise = cell(nx, MC_run);
x = cell(MC_run, 1);
x_hat = x;
P_hat = x;
Time = zeros(1, SimL);
u = zeros(2, SimL);
landmark_detection_indicator = cell(nl, 1);

for i_M = 1:MC_run
    
    noise_index_extract = (i_M - 1) * noise_channels;
    
    % Generate Measurement Noise
    for j = 1:nl
        z_noise{j, i_M} = randn(indep_noise{noise_index_extract + j}, SimL, 1);
    end
    
end

for i_M = 1:MC_run    
    % Initial States
    x_pv = [0, 0, 0.01]';
    x{i_M}(:, 1) = [x_pv; xl_true'];
    
    rng default
    xhat_pv = mvnrnd(x_pv, Ppv)';
    x_hat{i_M}(:, 1) = [xhat_pv; xl_hat'];
    
    % Estimation Error Covariance
    P_hat{i_M}(:, :, 1) = blkdiag(Ppv, Pl{:});
    
    % Process Noise Covariance
    Q = blkdiag(Qpv, Qi{:});
    
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
        x{i_M}(1, k + 1) = x{i_M}(1, k) + T*u(1, k)*cos(x{i_M}(3, k)) + w_x(k);
        x{i_M}(2, k + 1) = x{i_M}(2, k) + T*u(1, k)*sin(x{i_M}(3, k)) + w_y(k);
        x{i_M}(3, k + 1) = x{i_M}(3, k) + T*u(2, k) + w_theta(k);
        
        % Propogation Step
        xhat_bar = zeros(73, 1);
        xhat_bar(1) = x_hat{i_M}(1, k) + T*u(1, k)*cos(x_hat{i_M}(3, k));
        xhat_bar(2) = x_hat{i_M}(2, k) + T*u(1, k)*sin(x_hat{i_M}(3, k));
        xhat_bar(3) = x_hat{i_M}(3, k) + T*u(2, k);
        xhat_bar(4:73) = x_hat{i_M}(4:73);
        Fpv = Fk(x_hat{i_M}(:, k), u(:, k));
        F = blkdiag(Fpv, Fi{:});
        P_bar = F * P_hat{i_M}(:, :, k) * F' + Q;
        
        % Preallocate
        Hk = [];
        zk_hat = [];
        zk = [];
        R = [];
        
        for ii = 1:nl
            % Detect Landmarks in the Measurment Zone
            range_to_landmark_i = norm(x{i_M}(1:2, k + 1) - Xl{ii});
            
            if range_to_landmark_i <= m_r
                % Landmark Index
                ilx = (3 + 2*ii) - 1;
                ily = ilx + 1;
                
                % True Measurments
                zk_i = [h(x{i_M}(:, k + 1), Xl{ii}), phi(x{i_M}(:, k + 1), Xl{ii})]';
                zk = [zk; zk_i];
                
                % Estimated Measurements
                zk_hati = [h(xhat_bar(1:3), xhat_bar(ilx:ily)), phi(xhat_bar(1:3), xhat_bar(ilx:ily))]';
                zk_hat = [zk_hat; zk_hati];
                
                % Linearized Observation Jacobian
                HV = Hv(xhat_bar(1:3), xhat_bar(ilx:ily));                                              % Vehicle
                HL = [zeros(2, 2*(ii - 1)), Hl(xhat_bar(1:3), xhat_bar(ilx:ily)), zeros(2, 73 - ily)];  % Landmark(s)
                Hk = [Hk; [HV, HL]];
                
                % Measurement Noise Covariance
                R{ii} = Ri;
                
                % Indicator
                landmark_detection_indicator{ii}(k) = 1;
                
            else
                % Indicator
                landmark_detection_indicator{ii}(k) = 0;
                
            end
        end
        
        % Determine if Available Measurements
        if isempty(zk) == 0
            % Measurment Noise
            rng(k);
            Rk = blkdiag(R{:});
            r = sqrtm(Rk);
            vk = r*randn(length(zk), 1);
            
            % Correction
            v = (zk + vk) - zk_hat;
            S = Hk * P_bar * Hk' + Rk;
            Kk = P_bar * Hk' * inv(S);
            x_hat{i_M}(:, k + 1) = xhat_bar + Kk * v;
            P_hat{i_M}(:, :, k + 1) = P_bar - Kk * S * Kk';
            
        elseif isempty(zk) == 1
            % Propogation = Correction
            x_hat{i_M}(:, k + 1) = xhat_bar;
            P_hat{i_M}(:, :, k + 1) = P_bar;
            
        end
    end
end

%% RMSE Plot:
% Compute RMSE of Each State
for i = 1:nx
    for k = 1:(SimL + 1)
        temp(i, k) = 0;
        
        for i_M = 1:MC_run
            error{i_M}(i, k) = (x{i_M}(i, k) - x_hat{i_M}(i, k))^2;
            temp(i, k) = temp(i,k) + error{i_M}(i, k);
            
        end
        % RMSE
        RMSE(i, k) = sqrt(temp(i, k)/MC_run);
        
    end
end

% Plot RMSE (No Individual Runs)
tit = [num2str(MC_run), ' Monte Carlo runs'];
set(gcf, 'Position', [0, 0, 1000, 1000]);     % Define Figure Window Size

for i_plot = 1:3
    subplot(3, 1, i_plot)
    %     plot(time, RMSE(i_plot, :), 'r.', 'MarkerSize', 10);
    plot(Time, RMSE(i_plot, :), 'r', 'linewidth', 2.5);
    xlabel('time');
    y_temp = ['RMSE_', num2str(i_plot)]; ylabel(y_temp);
    grid on;
    set(findobj(gcf, 'type', 'axes'), 'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'Bold', 'LineWidth', 2);
    legend('RMSE', 'location', 'best');
    title(tit);
end

