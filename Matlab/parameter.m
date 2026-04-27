% Load Data
data = readtable("data/1Nut.csv");

% Parameters
BOX_MASS = 0.155;
NUT_MASS = 0.038;
NUT_NUM = 1;

m = BOX_MASS + (NUT_MASS * NUT_NUM);
x0 = data.kf_distance_4_(1);
x_eq = data.kf_distance_4_(end);

K = 25.119;    % Spring Constant
C = 0.001;    % Damping Coefficient

% Extract collected data
N = height(data);              % cleaner than size()

% Use first half of data
idx = 50:floor(N/2)-200;

Dist = data.kf_distance_4_(idx) - x_eq;

% Time vector (match data length!)
dt = 0.001;
Time = (0:length(idx)-1)' * dt;

% Plot
figure('Name','Response','Color','white')
plot(Time,Dist,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Displacement (m)')
title('System Response')