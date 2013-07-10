close all;
clear;
clc;

format bank;
format compact;

rf_data_raw = csvread('rh_rf_pressure.csv');
lf_data_raw = csvread('rh_lf_pressure.csv');
rf_data_dt = csvread('rh_rf_derivative.csv');
lf_data_dt = csvread('rh_lf_derivative.csv');

rf_x = linspace(1,size(rf_data_raw,1),size(rf_data_raw,1));
lf_x = linspace(1,size(lf_data_raw,1),size(lf_data_raw,1));

%% RF_CALIBRATED_DATA

figure(1);

subplot(2,1,1);
title('Right Finger Pressure');
hold all;

% Plot Without Specific Colors
% plot(B, data_raw);

% Plot with Specific Colors
plot(rf_x, rf_data_raw(:, 19), 'Color', [255/255, 153/255, 153/255]);
plot(rf_x, rf_data_raw(:, 20), 'Color', [255/255, 102/255. 102/255]);
plot(rf_x, rf_data_raw(:, 21), 'Color', [255/255, 000/255. 000/255]);

plot(rf_x, rf_data_raw(:, 16), 'Color', [255/255, 204/255, 153/255]);
plot(rf_x, rf_data_raw(:, 17), 'Color', [255/255, 178/255. 102/255]);
plot(rf_x, rf_data_raw(:, 18), 'Color', [255/255, 128/255. 000/255]);

plot(rf_x, rf_data_raw(:, 13), 'Color', [153/255, 255/255, 153/255]);
plot(rf_x, rf_data_raw(:, 14), 'Color', [102/255, 255/255. 102/255]);
plot(rf_x, rf_data_raw(:, 15), 'Color', [000/255, 255/255. 000/255]);

plot(rf_x, rf_data_raw(:, 10), 'Color', [153/255, 204/255, 255/255]);
plot(rf_x, rf_data_raw(:, 11), 'Color', [102/255, 178/255. 255/255]);
plot(rf_x, rf_data_raw(:, 12), 'Color', [000/255, 000/255. 255/255]);

plot(rf_x, rf_data_raw(:, 7), 'Color', [204/255, 153/255, 255/255]);
plot(rf_x, rf_data_raw(:, 8), 'Color', [178/255, 102/255. 255/255]);
plot(rf_x, rf_data_raw(:, 9), 'Color', [127/255, 000/255. 255/255]);

%% LF_CALIBRATED_DATA

subplot(2,1,2);
title('L Finger Pressure');
hold all;

% Plot Without Specific Colors
% plot(B, data_raw);

% Plot with Specific Colors
plot(lf_x, lf_data_raw(:, 19), 'Color', [255/255, 153/255, 153/255]);
plot(lf_x, lf_data_raw(:, 20), 'Color', [255/255, 102/255. 102/255]);
plot(lf_x, lf_data_raw(:, 21), 'Color', [255/255, 000/255. 000/255]);

plot(lf_x, lf_data_raw(:, 16), 'Color', [255/255, 204/255, 153/255]);
plot(lf_x, lf_data_raw(:, 17), 'Color', [255/255, 178/255. 102/255]);
plot(lf_x, lf_data_raw(:, 18), 'Color', [255/255, 128/255. 000/255]);

plot(lf_x, lf_data_raw(:, 13), 'Color', [153/255, 255/255, 153/255]);
plot(lf_x, lf_data_raw(:, 14), 'Color', [102/255, 255/255. 102/255]);
plot(lf_x, lf_data_raw(:, 15), 'Color', [000/255, 255/255. 000/255]);

plot(lf_x, lf_data_raw(:, 10), 'Color', [153/255, 204/255, 255/255]);
plot(lf_x, lf_data_raw(:, 11), 'Color', [102/255, 178/255. 255/255]);
plot(lf_x, lf_data_raw(:, 12), 'Color', [000/255, 000/255. 255/255]);

plot(lf_x, lf_data_raw(:, 7), 'Color', [204/255, 153/255, 255/255]);
plot(lf_x, lf_data_raw(:, 8), 'Color', [178/255, 102/255. 255/255]);
plot(lf_x, lf_data_raw(:, 9), 'Color', [127/255, 000/255. 255/255]);

% %% Calibrated Difference
% subplot(3,1,3);
% hold all;
% title('Finger Pressure Difference');
% 
% % Plot with Specific Colors
% 
% diff = max(rf_data_raw(:, 7:19) - lf_data_raw(:, 7:19), [], 2);
% 
% plot(rf_x, diff, 'Color', [255/255, 153/255, 153/255]);
%% RF_Derivative

figure(2);

subplot(2,1,1);
grid on;
title('Right Finger Derivative');
hold all

rf_dx = linspace(1,size(rf_data_dt,1),size(rf_data_dt,1));

% Plot with Specific Colors
plot(rf_dx, rf_data_dt(:, 19), 'Color', [255/255, 153/255, 153/255]);
plot(rf_dx, rf_data_dt(:, 20), 'Color', [255/255, 102/255. 102/255]);
plot(rf_dx, rf_data_dt(:, 21), 'Color', [255/255, 000/255. 000/255]);

plot(rf_dx, rf_data_dt(:, 16), 'Color', [255/255, 204/255, 153/255]);
plot(rf_dx, rf_data_dt(:, 17), 'Color', [255/255, 178/255. 102/255]);
plot(rf_dx, rf_data_dt(:, 18), 'Color', [255/255, 128/255. 000/255]);

plot(rf_dx, rf_data_dt(:, 13), 'Color', [153/255, 255/255, 153/255]);
plot(rf_dx, rf_data_dt(:, 14), 'Color', [102/255, 255/255. 102/255]);
plot(rf_dx, rf_data_dt(:, 15), 'Color', [000/255, 255/255. 000/255]);

plot(rf_dx, rf_data_dt(:, 10), 'Color', [153/255, 204/255, 255/255]);
plot(rf_dx, rf_data_dt(:, 11), 'Color', [102/255, 178/255. 255/255]);
plot(rf_dx, rf_data_dt(:, 12), 'Color', [000/255, 000/255. 255/255]);

plot(rf_dx, rf_data_dt(:, 7), 'Color', [204/255, 153/255, 255/255]);
plot(rf_dx, rf_data_dt(:, 8), 'Color', [178/255, 102/255. 255/255]);
plot(rf_dx, rf_data_dt(:, 9), 'Color', [127/255, 000/255. 255/255]);

%% LF_Derivative

subplot(2,1,2);
grid on;
title('Left Finger Derivative');
hold all

lf_dx = linspace(1,size(lf_data_dt,1),size(lf_data_dt,1));

% Plot with Specific Colors
plot(lf_dx, lf_data_dt(:, 19), 'Color', [255/255, 153/255, 153/255]);
plot(lf_dx, lf_data_dt(:, 20), 'Color', [255/255, 102/255. 102/255]);
plot(lf_dx, lf_data_dt(:, 21), 'Color', [255/255, 000/255. 000/255]);

plot(lf_dx, lf_data_dt(:, 16), 'Color', [255/255, 204/255, 153/255]);
plot(lf_dx, lf_data_dt(:, 17), 'Color', [255/255, 178/255. 102/255]);
plot(lf_dx, lf_data_dt(:, 18), 'Color', [255/255, 128/255. 000/255]);

plot(lf_dx, lf_data_dt(:, 13), 'Color', [153/255, 255/255, 153/255]);
plot(lf_dx, lf_data_dt(:, 14), 'Color', [102/255, 255/255. 102/255]);
plot(lf_dx, lf_data_dt(:, 15), 'Color', [000/255, 255/255. 000/255]);

plot(lf_dx, lf_data_dt(:, 10), 'Color', [153/255, 204/255, 255/255]);
plot(lf_dx, lf_data_dt(:, 11), 'Color', [102/255, 178/255. 255/255]);
plot(lf_dx, lf_data_dt(:, 12), 'Color', [000/255, 000/255. 255/255]);

plot(lf_dx, lf_data_dt(:, 7), 'Color', [204/255, 153/255, 255/255]);
plot(lf_dx, lf_data_dt(:, 8), 'Color', [178/255, 102/255. 255/255]);
plot(lf_dx, lf_data_dt(:, 9), 'Color', [127/255, 000/255. 255/255]);


%% Deriv Difference

% subplot(3,1,3);
% grid on;
% title('Derivative Difference');
% hold all
% 
% % Plot with Specific Colors
% plot(rf_dx, max(rf_data_dt(:, 7:19)) - max(lf_data_dt(:, 7:19)), 'Color', [255/255, 153/255, 153/255]);


%% Run the next Script
%Moving_Average_Analysis