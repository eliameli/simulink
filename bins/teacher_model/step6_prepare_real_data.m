%% step6_prepare_real_data.m — реальные данные телефона B в осях и шаге,
%  которые ожидает блок БИНС преподавателя
%
% Блок БИНС (bins_model2/BINS, BINS1) считает, что "a" и "w" на входе -
% это ускорение/угловая скорость в ОСЯХ МАШИНЫ в конвенции этой модели:
%   X - вперёд (вдоль движения)
%   Y - вверх  (гравитация = [0,-10,0], то есть вертикаль - именно Y)
%   Z - вбок   (куда именно - проверим по факту ниже)
% (разобрали это на шаге 4 структурного анализа - см. чат/README).
%
% А у телефона акселерометр/гироскоп пишут показания в СВОИХ, случайных
% осях - как телефон физически лежит в машине, так и считает. Значит,
% первым делом нужно найти, куда в осях телефона смотрят "вперёд",
% "вверх", "вбок" машины - и повернуть туда все показания.
%
% Это та же самая задача и то же решение, что мы уже делали в
% ../phoneB/step4_heading_and_mag.m (поиск продольной оси МНК по GPS) -
% тут просто переиспользуем ту же логику, без изменений в методе.

clear; clc; close all;

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Читаем сырые данные телефона B
% Accelerometer.csv у phyphox - ПОЛНЫЙ акселерометр, С гравитацией -
% это ровно то, что "видит" реальный прибор, и ровно то, что модель
% называет A_meas (измеренное ускорение).
accB = readtable(fullfile('..','data','phone_B_phyphox','Accelerometer.csv'), ...
    'VariableNamingRule','preserve');
t_acc = accB.("Time (s)");
a_raw = [accB.("X (m/s^2)"), accB.("Y (m/s^2)"), accB.("Z (m/s^2)")];   % сырые оси телефона

gyroB = readtable(fullfile('..','data','phone_B_phyphox','Gyroscope.csv'), ...
    'VariableNamingRule','preserve');
t_gyro = gyroB.("Time (s)");
w_raw = [gyroB.("X (rad/s)"), gyroB.("Y (rad/s)"), gyroB.("Z (rad/s)")];

linB = readtable(fullfile('..','data','phone_B_phyphox','Linear Accelerometer.csv'), ...
    'VariableNamingRule','preserve');
t_lin = linB.("Time (s)");
a_lin = [linB.("X (m/s^2)"), linB.("Y (m/s^2)"), linB.("Z (m/s^2)")];   % без гравитации - нужно для поиска осей

%% 2) GPS - только чтобы найти оси машины (курс и начальные условия считаем позже)
locB = readtable(fullfile('..','data','phone_B_phyphox','Location.csv'), ...
    'VariableNamingRule','preserve');
t_gps = locB.("Time (s)");
hacc_gps = locB.("Horizontal Accuracy (m)");
v_raw = locB.("Velocity (m/s)");
if iscell(v_raw), v_gps = str2double(v_raw); else, v_gps = v_raw; end
dir_raw = locB.("Direction (°)");
if iscell(dir_raw), dir_gps = str2double(dir_raw); else, dir_gps = dir_raw; end

good = hacc_gps < 20 & ~isnan(v_gps) & ~isnan(dir_gps);
t_good = t_gps(good);
v_good = v_gps(good);
dir_good = dir_gps(good);

dt_g = diff(t_good);
dv_g = diff(v_good);
keep = dt_g < 3;
dvdt_mid = dv_g(keep) ./ dt_g(keep);
t_mid = (t_good(1:end-1) + t_good(2:end)) / 2;
t_mid = t_mid(keep);

%% 3) "Вверх" телефона (из гравитации) и горизонтальный базис e1,e2
%     (один в один как в step4_heading_and_mag.m)
g_est = movmean(a_raw, 200);                        % медленная часть полного ускорения = гравитация
g_at_lin = interp1(t_acc, g_est, t_lin, 'linear', 'extrap');
up_lin = -g_at_lin ./ vecnorm(g_at_lin, 2, 2);

a_vert = sum(a_lin .* up_lin, 2) .* up_lin;
a_horiz = a_lin - a_vert;

u_mean = mean(up_lin, 1);
u_mean = u_mean / norm(u_mean);

ref = [1 0 0];
if abs(sum(ref .* u_mean)) > 0.9
    ref = [0 1 0];
end
e1 = ref - sum(ref .* u_mean) * u_mean;  e1 = e1 / norm(e1);
e2 = cross(u_mean, e1);

a_horiz_e1 = sum(a_horiz .* e1, 2);
a_horiz_e2 = sum(a_horiz .* e2, 2);

g_at_gyro = interp1(t_acc, g_est, t_gyro, 'linear', 'extrap');
up_gyro = -g_at_gyro ./ vecnorm(g_at_gyro, 2, 2);
wz = sum(w_raw .* up_gyro, 2);

%% 4) Продольная ось "вперёд" методом наименьших квадратов (та же модель: a_гор = (dv/dt)*f + (v*wz)*l)
v_i = interp1(t_good, v_good, t_lin, 'linear', NaN);
dvdt_i = interp1(t_mid, dvdt_mid, t_lin, 'linear', NaN);
wz_i = interp1(t_gyro, wz, t_lin, 'linear', NaN);

valid = ~isnan(v_i) & ~isnan(dvdt_i) & ~isnan(wz_i);
X = [dvdt_i(valid), v_i(valid) .* wz_i(valid)];
coef1 = X \ a_horiz_e1(valid);
coef2 = X \ a_horiz_e2(valid);

f_vec = coef1(1)*e1 + coef2(1)*e2;
f_unit = f_vec / norm(f_vec);

fprintf('Продольная ось "вперёд" в осях телефона: [%.3f %.3f %.3f]\n', f_unit);
fprintf('"Вверх" в осях телефона: [%.3f %.3f %.3f]\n', u_mean);

%% 5) Строим ЧИСТЫЙ (точно ортонормированный) базис машины и поворачиваем данные
% forward = f_unit, up = u_mean (из гравитации - самый надёжный источник),
% left - достраиваем крест-произведением, чтобы базис был ровно
% ортогональным (МНК-вектор l_unit из шага 4 не идеально перпендикулярен
% f_unit/up - а для матрицы поворота это обязательно).
up_unit = u_mean(:)';
fwd_unit = f_unit(:)';
left_unit = cross(up_unit, fwd_unit);
left_unit = left_unit / norm(left_unit);

% C: столбцы - оси машины (вперёд, вверх, влево), выраженные в осях
% телефона. Поворот сырых данных в оси машины: v_машина = C' * v_телефон
C = [fwd_unit', up_unit', left_unit'];

a_vehicle = a_raw * C;   % (N x 3) * (3 x 3) - то же самое, что (C' * v')' построчно
w_vehicle = w_raw * C;

fprintf('Угол между "вперёд" и "вверх" (должен быть ~90°%%): %.2f°\n', ...
    rad2deg(acos(max(min(sum(fwd_unit.*up_unit),1),-1))));

%% 6) Общий шаг времени 0.01 с (как требует задание) и ресэмплинг
t0 = max(t_acc(1), t_gyro(1));
t1 = min(t_acc(end), t_gyro(end));
t_fixed = (t0:0.01:t1)';

a_fixed = interp1(t_acc, a_vehicle, t_fixed, 'linear');
w_fixed = interp1(t_gyro, w_vehicle, t_fixed, 'linear');

fprintf('Готово: %d отсчётов на шаге 0.01 с, с %.2f по %.2f с\n', ...
    numel(t_fixed), t_fixed(1), t_fixed(end));

%% 7) Графики - смотрим, что получилось, ДО того как куда-то это подавать
figure('Name', 'Телефон B - данные в осях машины, шаг 0.01с');

subplot(2,1,1);
plot(t_fixed, a_fixed(:,1), 'r'); hold on;
plot(t_fixed, a_fixed(:,2), 'g');
plot(t_fixed, a_fixed(:,3), 'b');
grid on; xlabel('время, с'); ylabel('ускорение, м/с^2');
legend('вперёд (X)', 'вверх (Y)', 'влево (Z)', 'Location', 'best');
title('Ускорение в осях машины - ось "вверх" в покое должна быть ~ +10 (гравитация)');

subplot(2,1,2);
plot(t_fixed, rad2deg(w_fixed(:,1)), 'r'); hold on;
plot(t_fixed, rad2deg(w_fixed(:,2)), 'g');
plot(t_fixed, rad2deg(w_fixed(:,3)), 'b');
grid on; xlabel('время, с'); ylabel('угловая скорость, град/с');
legend('вокруг "вперёд" (крен)', 'вокруг "вверх" (курс)', 'вокруг "влево" (тангаж)', 'Location', 'best');
title('Угловая скорость в осях машины');

saveas(gcf, fullfile('..','figs','step6_vehicle_axes.png'));

save('step6_real_data.mat', 't_fixed', 'a_fixed', 'w_fixed', 'C', 'up_unit', 'fwd_unit', 'left_unit', ...
    't_good', 'v_good', 'dir_good');
