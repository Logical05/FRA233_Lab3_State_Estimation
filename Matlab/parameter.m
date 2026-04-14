%{  
This script for prepare data and parameters for parameter estimator.
1. Load your collected data to MATLAB workspace.
2. Run this script.
3. Follow parameter estimator instruction.
%}

% Spring Constant (K) from experiment
K = 0.625;
m = 0.1;
x0 = 18;
% Optimization's parameters
C = 0.1;     % Damping Coefficient

% Load Data
data = readtable("Data.csv");

% Extract collected data
Time = data.x;
Dist = data.y;

% Plot 
figure(Name='Response')
plot(Time,Dist)