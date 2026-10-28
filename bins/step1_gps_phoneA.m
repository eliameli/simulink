%% step1_gps_phoneA.m — GPS-трек телефона A
%
% Отдельный файл для GPS, чтобы не раздувать step1_read_and_look.m.
% Читает Location.csv телефона A (Sensor Logger) и строит трек на
% плоскости широта/долгота + график точности во времени.
%
% ВАЖНО (из задания): у телефона A speed и bearing всегда = -1 (нет
% значения - GPS-чип это просто не посчитал), поэтому красить точки по
% скорости, как это сделаем для телефона B, здесь нельзя - вместо этого
% красим по horizontalAccuracy (точности определения координат), чтобы
% сразу увидеть разброс качества. По заданию этот GPS "не годится как
% эталон": всего 106 точек за 574 с (то есть реже раза в секунду) и
% медианная точность ~65 м - для сравнения, ширина дороги метров 7-10.
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/)

clear; clc; close all;

if ~exist('figs', 'dir')
    mkdir('figs');
end

%% 1) Чтение файла
loc_file = fullfile('data', 'phone_A_sensorlogger', 'Location.csv');
L = readtable(loc_file);

t_s   = L.seconds_elapsed;       % время от начала записи, секунды
lat   = L.latitude;              % широта, градусы
lon   = L.longitude;             % долгота, градусы
hacc  = L.horizontalAccuracy;    % точность по горизонтали, метры (чем меньше - тем лучше)

%% 2) Статистика
fprintf('Телефон A, Location.csv:\n');
fprintf('  точек: %d\n', height(L));
fprintf('  длительность записи: %.1f с\n', t_s(end) - t_s(1));
fprintf('  точность (horizontalAccuracy): медиана %.1f м, мин %.1f м, макс %.1f м\n', ...
    median(hacc), min(hacc), max(hacc));

%% 3) Графики: трек + точность во времени, 1 строка x 2 столбца
figure('Name', 'А - GPS (Location.csv)');

subplot(1,2,1);
% x = долгота, y = широта - так привычнее читать как "карту" (восток
% вправо, север вверх), хотя масштаб по осям тут ещё не в метрах (это
% будет в шаге 2 - перевод в декартовы координаты).
scatter(lon, lat, 25, hacc, 'filled');
colorbar;
colormap(gca, 'parula');
grid on; axis equal;
xlabel('долгота, °'); ylabel('широта, °');
title('GPS-трек, цвет = точность, м (меньше = лучше)');

subplot(1,2,2);
plot(t_s, hacc, 'o-');
grid on;
xlabel('время, с'); ylabel('horizontalAccuracy, м');
title('Точность GPS во времени - видно, насколько она "гуляет"');

saveas(gcf, fullfile('figs', 'step1_phoneA_gps.png'));
