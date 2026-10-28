%% step1_gps_phoneA.m — GPS-трек телефона A
%
% Отдельный файл для GPS. Читает Location.csv телефона A (Sensor Logger)
% и строит простой график координат - широта от долготы.
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/)

clear; clc; close all;

%% 1) Чтение файла
loc_file = fullfile('data', 'phone_A_sensorlogger', 'Location.csv');
L = readtable(loc_file);

lat = L.latitude;    % широта, градусы
lon = L.longitude;    % долгота, градусы

%% 2) График
% x = долгота, y = широта - так привычнее читать как "карту" (восток
% вправо, север вверх).
figure('Name', 'А - GPS (Location.csv)');
plot(lon, lat, 'o-');
grid on; axis equal;
xlabel('долгота, °'); ylabel('широта, °');
title('Телефон A: GPS-трек');
