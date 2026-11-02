%% step1_LinearAccel.m — акселерометр телефона B БЕЗ гравитации
%

% Linear Accelerometer.csv - iOS сама вычла гравитацию 
% Единицы  м/с^2.


clear; clc; close all;

%% 1) Чтение файла
accel_file = fullfile('..', 'data', 'phone_B_phyphox', 'Linear Accelerometer.csv');
T = readtable(accel_file, 'VariableNamingRule', 'preserve');

t_s = T.("Time (s)");
ax  = T.("X (m/s^2)");
ay  = T.("Y (m/s^2)");
az  = T.("Z (m/s^2)");

% Без гравитации - значит, в покое все три оси и модуль должны быть
% близки к нулю (в отличие от Accelerometer.csv, где в покое ~9.81).
a_norm = sqrt(ax.^2 + ay.^2 + az.^2);

%% 2) Графики: 
figure('Name', 'B - Linear Accelerometer (без гравитации)');

subplot(3,2,1);
plot(t_s, ax, 'r', t_s, ay, 'g', t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('ускорение, м/с^2');
legend('x', 'y', 'z', 'Location', 'best');
title('Все три оси вместе');

subplot(3,2,2);
plot(t_s, ax, 'r');
grid on; xlabel('время, с'); ylabel('a_x, м/с^2');
title('Ось x');

subplot(3,2,3);
plot(t_s, ay, 'g');
grid on; xlabel('время, с'); ylabel('a_y, м/с^2');
title('Ось y');

subplot(3,2,4);
plot(t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('a_z, м/с^2');
title('Ось z');

subplot(3,2,5);
plot(t_s, a_norm, 'k');
grid on; xlabel('время, с'); ylabel('|a|, м/с^2');
title('Модуль |a| - в покое ожидаем ~0 (гравитация уже вычтена)');
