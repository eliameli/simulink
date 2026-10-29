%% step1_gps.m — GPS-трек телефона B (phyphox)
%

% ВАЖНО : в Location.csv у phyphox пропуски (когда GPS ещё
% не поймал скорость/направление) записаны текстом "не число", а не
% пустой ячейкой - из-за этого колонки Velocity и Direction читаются не
% как числа, а как текст (cell-массив строк). Переводим в числа через
% str2double - для настоящих чисел сработает как обычно, а "не число"
% превратится в NaN (это то же самое, что pd.to_numeric(errors="coerce")
% в Python, просто в MATLAB).

% ??????? РАЗОБРАТЬСЯ ПОЧЕМУ ПИШЕТ NAN
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
