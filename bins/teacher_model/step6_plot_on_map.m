%% step6_plot_on_map.m — трек GPS и БИНС поверх графа дорог OSM
%
% Карта (map.osm) и парсер (read_osm_roads.m) лежат в osm_map/ рядом.
% Чтобы всё совпало по координатам, граф дорог переводим в метры
% ОТНОСИТЕЛЬНО ТОЙ ЖЕ опорной точки (lat0, lon0), что и GPS/БИНС в
% step6_run_bins.m - там это среднее по "хорошим" GPS-точкам поездки.

clear; clc; close all;

addpath('osm_map');

load('step6_run_bins_result.mat');   % gps_east, gps_north, bins_east_aligned, bins_north_aligned,
                                      % gps_east_at_anchor, gps_north_at_anchor, lat0, lon0 (в радианах), t_gps_good, err_dist

% read_osm_roads ждёт lat0/lon0 в ГРАДУСАХ (формула внутри неё на cosd),
% а у нас они в радианах (остались от ECEF-расчёта) - переводим.
lat0_deg = rad2deg(lat0);
lon0_deg = rad2deg(lon0);

R = read_osm_roads(fullfile('osm_map', 'map.osm'), lat0_deg, lon0_deg);

%% График: дороги серым, поверх - GPS и БИНС
figure('Name', 'БИНС и GPS поверх карты дорог');

% отрезки дорог - рисуем все разом через NaN-разделители (быстрее, чем
% цикл с отдельным plot на каждый отрезок)
X = [R.x(R.s)'; R.x(R.t)'; nan(1, numel(R.s))];
Y = [R.y(R.s)'; R.y(R.t)'; nan(1, numel(R.s))];
plot(X(:), Y(:), '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5, 'DisplayName', 'дороги (OSM)');
hold on;

plot(gps_east, gps_north, 'k.-', 'LineWidth', 1.2, 'DisplayName', 'GPS');
plot(bins_east_aligned, bins_north_aligned, 'r-', 'LineWidth', 1.2, 'DisplayName', 'БИНС (реальные данные)');
plot(gps_east_at_anchor, gps_north_at_anchor, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'точка привязки курса');

grid on; axis equal; xlabel('восток, м'); ylabel('север, м');
legend('Location', 'best');
title('GPS и БИНС поверх графа дорог (общая опорная точка)');

saveas(gcf, fullfile('..', 'figs', 'step6_on_map.png'));
