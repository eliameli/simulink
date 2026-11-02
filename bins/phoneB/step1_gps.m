%% step1_gps.m — GPS-трек телефона B (phyphox)
%
% Простой график координат - широта от долготы. Про "не число" и NaN в
% Velocity/Direction - см. step2_gps_to_xy.m, тут эти столбцы не нужны.

clear; clc; close all;

%% 1) Чтение файла
% 'VariableNamingRule','preserve' - чтобы имена столбцов остались как в
% файле ("Latitude (°)" и т.п.), а не обрезались MATLAB непредсказуемо.
loc_file = fullfile('..', 'data', 'phone_B_phyphox', 'Location.csv');
L = readtable(loc_file, 'VariableNamingRule', 'preserve');

lat = L.("Latitude (°)");     % широта, градусы - это обычные числа, без пропусков
lon = L.("Longitude (°)");    % долгота, градусы

%% 2) График трека
figure('Name', 'B - GPS (Location.csv)');
plot(lon, lat, 'o-');
grid on; axis equal;
xlabel('долгота, °'); ylabel('широта, °');
title('Телефон B: GPS-трек');
