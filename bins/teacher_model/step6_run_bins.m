%% step6_run_bins.m — подаём реальные данные телефона B в блок БИНС (BINS.mdl)
%
% BINS.mdl - это "чистый" блок БИНС из модели преподавателя, без модели
% движения и без модели датчика (ровно то, что просит задание на шаге 6:
% "подать данные в блок БИНС, минуя модель движения и модель датчика").
%
% Входные порты (проверено чтением .mdl, номера портов - не угадано):
%   1 = a             (м/с^2, в осях машины)
%   2 = w             (рад/с, в осях машины)
%   3 = start r       (начальное положение, земные оси)
%   4 = start v       (начальная скорость, земные оси)
%   5 = start angles  (начальные углы [тета; псi; гамма])
% Выходные порты: 1 = r_e, 2 = v_e, 3 = ang

clear; clc; close all;

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

load('step6_initial_conditions.mat');   % t_run, a_run, w_run, theta0, psi0, gamma0, v0, r0, t_course_anchor
load('step6_real_data.mat', 't_fixed');  % нужен только t_fixed(1) для пересчёта t_course_anchor во "время прогона"

%% 1) Собираем единый входной сигнал для корневых портов (в порядке 1..5)
n = numel(t_run);
start_r_rep      = repmat(r0,            n, 1);
start_v_rep      = repmat(v0,            n, 1);
start_angles_rep = repmat([theta0, psi0, gamma0], n, 1);

u = [t_run, a_run, w_run, start_r_rep, start_v_rep, start_angles_rep];
% столбцы: t | a(3) | w(3) | start_r(3) | start_v(3) | start_angles(3)

%% 2) Настраиваем и запускаем модель
mdl = 'BINS';
load_system(mdl);

set_param(mdl, 'LoadExternalInput', 'on', 'ExternalInput', 'u');
set_param(mdl, 'StopTime', num2str(t_run(end)));
set_param(mdl, 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Array');
set_param(mdl, 'SaveTime', 'on', 'TimeSaveName', 'tout');

simOut = sim(mdl);

t_out = simOut.get('tout');
y_out = simOut.get('yout');   % столбцы: r_e(1:3) | v_e(4:6) | ang(7:9)

r_e = y_out(:, 1:3);
v_e = y_out(:, 4:6);
ang = y_out(:, 7:9);

fprintf('БИНС прогнан: %d отсчётов, %.1f с\n', numel(t_out), t_out(end));

%% 3) GPS в метрах (та же логика, что в step2_gps_to_xy.m) - для сравнения
locB = readtable(fullfile('..','data','phone_B_phyphox','Location.csv'), 'VariableNamingRule','preserve');
t_gps = locB.("Time (s)");
lat_deg = locB.("Latitude (°)");
lon_deg = locB.("Longitude (°)");
h_gps   = locB.("Height (m)");
hacc_gps = locB.("Horizontal Accuracy (m)");
v_raw = locB.("Velocity (m/s)");
if iscell(v_raw), v_check = str2double(v_raw); else, v_check = v_raw; end

good = hacc_gps < 20 & ~isnan(v_check);
t_gps_good = t_gps(good);
lat_good = deg2rad(lat_deg(good));
lon_good = deg2rad(lon_deg(good));
h_good   = h_gps(good);

lat0 = mean(lat_good); lon0 = mean(lon_good); h0 = mean(h_good);

a_wgs = 6378137; f_wgs = 1/298.257223563; e2 = 2*f_wgs - f_wgs^2;
N = @(phi) a_wgs ./ sqrt(1 - e2*sin(phi).^2);
ecef = @(phi,lam,hh) deal((N(phi)+hh).*cos(phi).*cos(lam), (N(phi)+hh).*cos(phi).*sin(lam), (N(phi).*(1-e2)+hh).*sin(phi));
[X,Y,Z] = ecef(lat_good, lon_good, h_good);
[X0,Y0,Z0] = ecef(lat0, lon0, h0);
dX=X-X0; dY=Y-Y0; dZ=Z-Z0;
gps_east  = -sin(lon0).*dX + cos(lon0).*dY;
gps_north = -sin(lat0).*cos(lon0).*dX - sin(lat0).*sin(lon0).*dY + cos(lat0).*dZ;

%% 4) Переводим r_e (земные оси БИНС: X=север, Z=восток - см. обсуждение в чате)
%     в те же (восток, север), и совмещаем с GPS в ОДНОЙ надёжной точке -
%     в момент t_course_anchor (где мы и брали курс), а не с нуля: там,
%     где обе траектории ещё должны совпадать по построению.
bins_north = r_e(:,1);
bins_east  = r_e(:,3);

t_anchor_run = t_course_anchor - t_fixed(1);   % тот же момент, но от начала прогона (t_run начинается с 0)

gps_east_at_anchor  = interp1(t_gps_good, gps_east,  t_course_anchor);
gps_north_at_anchor = interp1(t_gps_good, gps_north, t_course_anchor);
bins_east_at_anchor  = interp1(t_out, bins_east,  t_anchor_run);
bins_north_at_anchor = interp1(t_out, bins_north, t_anchor_run);

offset_east  = gps_east_at_anchor  - bins_east_at_anchor;
offset_north = gps_north_at_anchor - bins_north_at_anchor;

bins_east_aligned  = bins_east  + offset_east;
bins_north_aligned = bins_north + offset_north;

%% 5) Ошибка во времени (расстояние БИНС-GPS, только там, где GPS "хороший")
bins_east_at_gps  = interp1(t_out, bins_east_aligned,  t_gps_good - t_fixed(1), 'linear', NaN);
bins_north_at_gps = interp1(t_out, bins_north_aligned, t_gps_good - t_fixed(1), 'linear', NaN);
err_dist = hypot(bins_east_at_gps - gps_east, bins_north_at_gps - gps_north);

for thresh = [10 100 1000]
    i_cross = find(err_dist > thresh, 1, 'first');
    if isempty(i_cross)
        fprintf('Ошибка не достигла %d м за всю запись\n', thresh);
    else
        fprintf('Ошибка достигает %d м примерно на t = %.1f с\n', thresh, t_gps_good(i_cross));
    end
end

%% 6) Графики
figure('Name', 'БИНС на реальных данных vs GPS');

subplot(1,2,1);
plot(gps_east, gps_north, 'k.-', 'DisplayName', 'GPS'); hold on;
plot(bins_east_aligned, bins_north_aligned, 'r-', 'DisplayName', 'БИНС (реальные данные)');
plot(gps_east_at_anchor, gps_north_at_anchor, 'bo', 'MarkerFaceColor','b', 'DisplayName', 'точка привязки курса');
grid on; axis equal; xlabel('восток, м'); ylabel('север, м');
legend('Location','best');
title('Траектория: БИНС vs GPS');

subplot(1,2,2);
plot(t_gps_good, err_dist, 'm');
grid on; xlabel('время, с'); ylabel('ошибка, м');
title('Расхождение БИНС-GPS во времени');

saveas(gcf, fullfile('..','figs','step6_bins_vs_gps.png'));
