%% step2_gps_to_xy.m — GPS (широта/долгота) -> декартовы координаты x,y
%
% Телефон B, Location.csv. Та же математика, что и у телефона A
% (bins/phoneA/step2_gps_to_xy.m) - два способа перевода в метры и
% оценка поправки на шарообразность Земли - см. комментарии к формулам
% там, здесь не повторяем.
%
% Отличие от телефона A: тут GPS - нормальный (это и есть эталон по
% заданию), поэтому дополнительно отбираем только "хорошие" точки
% (Horizontal Accuracy < 20 м и Velocity не NaN - критерий из задания).
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/phoneB/)

clear; clc; close all;

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Чтение GPS
loc_file = fullfile('..', 'data', 'phone_B_phyphox', 'Location.csv');
L = readtable(loc_file, 'VariableNamingRule', 'preserve');

lat_deg = L.("Latitude (°)");             % широта, градусы
lon_deg = L.("Longitude (°)");            % долгота, градусы
h       = L.("Height (m)");               % высота, метры
hacc    = L.("Horizontal Accuracy (m)");  % точность по горизонтали, метры

% Velocity читается не как число, а как текст (cell-массив) - там, где
% GPS ещё не успел посчитать скорость, в файле стоит текст "не число"
% вместо пустой ячейки. str2double переводит текст в число, а там, где
% число не распознаётся (в т.ч. "не число"), ставит NaN.
velocity_raw = L.("Velocity (m/s)");
if iscell(velocity_raw)
    velocity = str2double(velocity_raw);
else
    velocity = velocity_raw;   % на случай, если MATLAB всё же распознал колонку как числовую
end

%% 2) Отбор "хороших" точек (критерий из задания)
good = hacc < 20 & ~isnan(velocity);
fprintf('Телефон B, Location.csv: всего точек %d, хороших %d (%.0f%%)\n', ...
    height(L), sum(good), 100*sum(good)/height(L));

lat_deg = lat_deg(good);
lon_deg = lon_deg(good);
h       = h(good);

%% 3) Опорная точка (начало координат) - среднее по хорошим точкам
lat0_deg = mean(lat_deg);
lon0_deg = mean(lon_deg);
h0       = mean(h);

lat  = deg2rad(lat_deg);   lat0 = deg2rad(lat0_deg);
lon  = deg2rad(lon_deg);   lon0 = deg2rad(lon0_deg);

%% 4) Способ 1 - точный, через ECEF и ENU (формулы - см. phoneA/step2_gps_to_xy.m)
a = 6378137;
f = 1 / 298.257223563;
e2 = 2*f - f^2;

N = @(phi) a ./ sqrt(1 - e2 * sin(phi).^2);
ecef = @(phi, lam, hh) deal( ...
    (N(phi) + hh) .* cos(phi) .* cos(lam), ...
    (N(phi) + hh) .* cos(phi) .* sin(lam), ...
    (N(phi) .* (1 - e2) + hh) .* sin(phi) );

[X, Y, Z]    = ecef(lat, lon, h);
[X0, Y0, Z0] = ecef(lat0, lon0, h0);

dX = X - X0; dY = Y - Y0; dZ = Z - Z0;

east_1  =  -sin(lon0).*dX + cos(lon0).*dY;
north_1 =  -sin(lat0).*cos(lon0).*dX - sin(lat0).*sin(lon0).*dY + cos(lat0).*dZ;

x1 = east_1;
y1 = north_1;

%% 5) Способ 2 - плоское приближение
R = 6371000;
x2 = R * cos(lat0) * (lon - lon0);
y2 = R * (lat - lat0);

%% 6) Сравнение способов
diff_xy = hypot(x1 - x2, y1 - y2);
fprintf('Способ 1 (точный) vs способ 2 (плоский):\n');
fprintf('  макс. расхождение: %.4f м\n', max(diff_xy));
fprintf('  среднее расхождение: %.4f м\n', mean(diff_xy));

%% 7) График
figure('Name', 'Телефон B - GPS в декартовых координатах');
plot(x1, y1, 'b.-', 'DisplayName', 'способ 1 (точный)'); hold on;
plot(x2, y2, 'ro--', 'DisplayName', 'способ 2 (плоский)');
grid on; axis equal;
xlabel('x (восток), м'); ylabel('y (север), м');
legend('Location', 'best');
title('Телефон B: трек в метрах (только "хорошие" точки)');

saveas(gcf, fullfile('..', 'figs', 'step2_phoneB_gps_xy.png'));
