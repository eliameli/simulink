%% step2_gps_on_map.m — трек GPS (телефон B) поверх графа дорог OSM
%
% То же самое чтение и перевод GPS в метры, что в step2_gps_to_xy.m
% (способ 1, точный, через ECEF/ENU), плюс поверх - дороги из OSM
% (map.osm, ../osm_map/), привязанные к ТОЙ ЖЕ опорной точке (lat0,lon0).

clear; clc; close all;

addpath(fullfile('..', 'osm_map'));

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Чтение GPS (телефон B, Location.csv) - как в step2_gps_to_xy.m
loc_file = fullfile('..', 'data', 'phone_B_phyphox', 'Location.csv');
L = readtable(loc_file, 'VariableNamingRule', 'preserve');

lat_deg = L.("Latitude (°)");
lon_deg = L.("Longitude (°)");
h       = L.("Height (m)");
hacc    = L.("Horizontal Accuracy (m)");

velocity_raw = L.("Velocity (m/s)");
if iscell(velocity_raw)
    velocity = str2double(velocity_raw);
else
    velocity = velocity_raw;
end

good = hacc < 20 & ~isnan(velocity);
fprintf('Телефон B, Location.csv: всего точек %d, хороших %d (%.0f%%)\n', ...
    height(L), sum(good), 100*sum(good)/height(L));

lat_deg = lat_deg(good);
lon_deg = lon_deg(good);
h       = h(good);

%% 2) Опорная точка - среднее по хорошим точкам (те же градусы,
%     что нужны read_osm_roads - без перевода в радианы и обратно)
lat0_deg = mean(lat_deg);
lon0_deg = mean(lon_deg);
h0       = mean(h);

lat  = deg2rad(lat_deg);   lat0 = deg2rad(lat0_deg);
lon  = deg2rad(lon_deg);   lon0 = deg2rad(lon0_deg);

%% 3) GPS -> метры, способ 1 (точный, ECEF -> ENU)
a_wgs = 6378137;
f_wgs = 1 / 298.257223563;
e2 = 2*f_wgs - f_wgs^2;

N = @(phi) a_wgs ./ sqrt(1 - e2 * sin(phi).^2);
ecef = @(phi, lam, hh) deal( ...
    (N(phi) + hh) .* cos(phi) .* cos(lam), ...
    (N(phi) + hh) .* cos(phi) .* sin(lam), ...
    (N(phi) .* (1 - e2) + hh) .* sin(phi) );

[X, Y, Z]    = ecef(lat, lon, h);
[X0, Y0, Z0] = ecef(lat0, lon0, h0);

dX = X - X0; dY = Y - Y0; dZ = Z - Z0;

gps_x =  -sin(lon0).*dX + cos(lon0).*dY;                                              % восток, м
gps_y =  -sin(lat0).*cos(lon0).*dX - sin(lat0).*sin(lon0).*dY + cos(lat0).*dZ;          % север, м

fprintf('GPS в метрах: x (восток) от %.1f до %.1f, y (север) от %.1f до %.1f\n', ...
    min(gps_x), max(gps_x), min(gps_y), max(gps_y));

%% 4) Дороги OSM, привязанные к ТОЙ ЖЕ опорной точке (в градусах - уже есть)
R = read_osm_roads(fullfile('..', 'osm_map', 'map.osm'), lat0_deg, lon0_deg);

fprintf('Дороги в метрах: x от %.1f до %.1f, y от %.1f до %.1f\n', ...
    min(R.x), max(R.x), min(R.y), max(R.y));
% если эти диапазоны не пересекаются с GPS-диапазоном выше - что-то не
% так с опорной точкой или с файлом карты, и график будет пустым/не тем

%% 5) График
figure('Name', 'GPS (телефон B) поверх карты дорог');

roadX = [R.x(R.s)'; R.x(R.t)'; nan(1, numel(R.s))];
roadY = [R.y(R.s)'; R.y(R.t)'; nan(1, numel(R.s))];
plot(roadX(:), roadY(:), '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5, ...
    'DisplayName', 'дороги (OSM)');
hold on;
plot(gps_x, gps_y, 'b.-', 'LineWidth', 1.2, 'DisplayName', 'GPS (телефон B)');

grid on; axis equal; xlabel('восток, м'); ylabel('север, м');
legend('Location', 'best');
title('Телефон B: трек GPS поверх графа дорог');

saveas(gcf, fullfile('..', 'figs', 'step2_phoneB_gps_on_map.png'));
