%% step4_heading_and_mag.m — ось машины, курс по гироскопу, магнитометр как датчик поворота
%
% Телефон B (у него нормальный GPS - используем его как эталон ТОЛЬКО
% для калибровки/проверки, не как вход в итоговый навигатор без GPS).
%
% Три части:
%   1) находим, куда направлена ось "вперёд машины" в осях телефона
%      (методом наименьших квадратов по GPS-скорости);
%   2) считаем курс интегрированием гироскопа, сверяем с GPS;
%   3) магнитометр как датчик ПОВОРОТА (не как компас) - показываем,
%      что абсолютный магнитный курс врёт, а вот его ИЗМЕНЕНИЕ неплохо
%      совпадает с гироскопом.
%


clear; clc; close all;

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Читаем всё, что нужно, и строим "вверх" + горизонтальный базис e1,e2
accB = readtable(fullfile('..', 'data', 'phone_B_phyphox', 'Accelerometer.csv'), ...
    'VariableNamingRule', 'preserve');
t_accB = accB.("Time (s)");
a_full_B = [accB.("X (m/s^2)"), accB.("Y (m/s^2)"), accB.("Z (m/s^2)")];
g_B_est = movmean(a_full_B, 200);   % оценка гравитации (та же логика, что в step3_sync.m)

linB = readtable(fullfile('..', 'data', 'phone_B_phyphox', 'Linear Accelerometer.csv'), ...
    'VariableNamingRule', 'preserve');
t_lin = linB.("Time (s)");
a_lin = [linB.("X (m/s^2)"), linB.("Y (m/s^2)"), linB.("Z (m/s^2)")];

g_at_lin = interp1(t_accB, g_B_est, t_lin, 'linear', 'extrap');
up_lin = -g_at_lin ./ vecnorm(g_at_lin, 2, 2);

a_vert = sum(a_lin .* up_lin, 2) .* up_lin;
a_horiz = a_lin - a_vert;   % горизонтальная (без гравитации, без вертикальной части) составляющая, вектор

% Фиксированный базис в горизонтальной плоскости (e1,e2): телефон жёстко
% закреплён в машине, поэтому "вверх" почти не меняется за поездку -
% берём среднее направление "вверх" и строим под него одну пару осей на
% всю запись, а не пересчитываем заново на каждом шаге.
u_mean = mean(up_lin, 1);
u_mean = u_mean / norm(u_mean);

ref = [1 0 0];
if abs(sum(ref .* u_mean)) > 0.9   % "вверх" почти совпал с выбранной осью - берём другую, чтобы не делить на ~0
    ref = [0 1 0];
end
e1 = ref - sum(ref .* u_mean) * u_mean;
e1 = e1 / norm(e1);
e2 = cross(u_mean, e1);   % по правилу правой руки - поворот от e1 к e2 вокруг "вверх" положительный (как и wz ниже)

a_horiz_e1 = sum(a_horiz .* e1, 2);   % проекции горизонтального ускорения на оси e1,e2 - уже просто числа, не векторы
a_horiz_e2 = sum(a_horiz .* e2, 2);

gyroB = readtable(fullfile('..', 'data', 'phone_B_phyphox', 'Gyroscope.csv'), ...
    'VariableNamingRule', 'preserve');
t_B = gyroB.("Time (s)");
w_B = [gyroB.("X (rad/s)"), gyroB.("Y (rad/s)"), gyroB.("Z (rad/s)")];
g_B_i = interp1(t_accB, g_B_est, t_B, 'linear', 'extrap');
up_B = -g_B_i ./ vecnorm(g_B_i, 2, 2);
wz_B = sum(w_B .* up_B, 2);              % скорость рысканья, рад/с (та же формула, что в step3_sync.m)
wz_B_smooth = movmean(wz_B, 20);

%% 2) GPS: "хорошие" точки, скорость и её производная
locB = readtable(fullfile('..', 'data', 'phone_B_phyphox', 'Location.csv'), ...
    'VariableNamingRule', 'preserve');
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

%% 3) Продольная ось машины в осях телефона (МНК)
% Модель: a_гор ≈ (dv/dt)*f + (v*wz)*l - продольное ускорение вдоль
% "вперёд" (f) плюс центростремительное ускорение вдоль "влево" (l).
% Переносим v, dv/dt и wz на моменты времени акселерометра (Linear
% Accelerometer.csv) - он и даёт нам a_гор.
v_i = interp1(t_good, v_good, t_lin, 'linear', NaN);
dvdt_i = interp1(t_mid, dvdt_mid, t_lin, 'linear', NaN);
wz_i = interp1(t_B, wz_B, t_lin, 'linear', NaN);

valid = ~isnan(v_i) & ~isnan(dvdt_i) & ~isnan(wz_i);
X = [dvdt_i(valid), v_i(valid) .* wz_i(valid)];   % регрессоры: dv/dt и v*wz, по столбцам

coef1 = X \ a_horiz_e1(valid);   % метод наименьших квадратов (обратная косая черта - решает X*coef=y в смысле МНК)
coef2 = X \ a_horiz_e2(valid);

f_2d = [coef1(1); coef2(1)];   % компоненты "вперёд" в базисе (e1,e2)
l_2d = [coef1(2); coef2(2)];   % компоненты "влево" в базисе (e1,e2)

f_vec = f_2d(1)*e1 + f_2d(2)*e2;   % возвращаем в полный 3D вектор в осях телефона
l_vec = l_2d(1)*e1 + l_2d(2)*e2;

f_unit = f_vec / norm(f_vec);
l_unit = l_vec / norm(l_vec);

angle_fl_deg = rad2deg(acos(max(min(sum(f_unit.*l_unit), 1), -1)));   % угол между f и l - проверка, что ~90°

fprintf('Продольная ось машины "вперёд" в осях телефона: [%.3f %.3f %.3f]\n', f_unit);
fprintf('Поперечная ось машины "влево" в осях телефона:  [%.3f %.3f %.3f]\n', l_unit);
fprintf('Угол между "вперёд" и "влево": %.1f° (должен быть ~90°, если МНК отработал нормально)\n', angle_fl_deg);

%% 4) Курс по гироскопу, сверка с GPS
% ВАЖНО: psi0 измерен GPS-ом в момент t_good(1) - а это НЕ момент начала
% записи (первые ~145 с GPS у телефона B неточный, см. задание, поэтому
% t_good(1) наступает позже). cumtrapz же всегда считает интеграл от
% самого первого момента t_B(1). Если бы просто вычли psi0 из этого
% интеграла, курс оказался бы привязан не к той точке времени. Поэтому
% сначала находим, чему равен "сырой" интеграл именно в момент t_good(1)
% (через интерполяцию), и привязываем psi0 именно к нему.
psi0 = deg2rad(dir_good(1));                   % начальный курс - из первой "хорошей" точки GPS
raw_integral = cumtrapz(t_B, wz_B);             % интеграл угловой скорости от t_B(1) (произвольная точка отсчёта)
ref_integral = interp1(t_B, raw_integral, t_good(1));   % значение этого интеграла в момент t_good(1)
psi_gyro = psi0 - (raw_integral - ref_integral);         % теперь привязано к правильному моменту
psi_gyro_deg = mod(rad2deg(psi_gyro), 360);

figure('Name', 'B - курс: гироскоп vs GPS');
plot(t_B, psi_gyro_deg, 'r-'); hold on;
plot(t_good, dir_good, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
grid on; xlabel('время, с'); ylabel('курс, °(от севера, по часовой)');
legend('курс по гироскопу', 'курс по GPS (Direction)', 'Location', 'best');
title('Курс: интеграл гироскопа против GPS');

saveas(gcf, fullfile('..', 'figs', 'step4_heading_gyro_vs_gps.png'));

%% 5) Магнитометр как датчик поворота
magB = readtable(fullfile('..', 'data', 'phone_B_phyphox', 'Magnetometer.csv'), ...
    'VariableNamingRule', 'preserve');
t_mag = magB.("Time (s)");
m_B = [magB.("X (µT)"), magB.("Y (µT)"), magB.("Z (µT)")];

g_at_mag = interp1(t_accB, g_B_est, t_mag, 'linear', 'extrap');
up_mag = -g_at_mag ./ vecnorm(g_at_mag, 2, 2);

m_vert = sum(m_B .* up_mag, 2) .* up_mag;
m_horiz = m_B - m_vert;                 % магнитное поле, спроецированное на горизонталь
m_norm = vecnorm(m_B, 2, 2);            % модуль ПОЛНОГО поля (для отдельного графика ниже)

m_e1 = sum(m_horiz .* e1, 2);
m_e2 = sum(m_horiz .* e2, 2);

% Сглаживаем вектор поля ДО вычисления угла, а не после. Если этого не
% сделать: помехи (кузов, проезжающие машины - видно на нижнем графике
% |B| как резкие иглы) на миг почти обнуляют горизонтальную составляющую
% поля, а atan2 возле нуля крайне чувствителен к шуму - один шумный
% отсчёт даёт скачок угла на десятки градусов за один шаг по времени
% (~0.01 с), что после деления на dt превращается в нереальные тысячи
% градусов в секунду и полностью забивает корреляцию с гироскопом.
m_e1_smooth = movmean(m_e1, 20);
m_e2_smooth = movmean(m_e2, 20);

heading_mag = atan2(m_e2_smooth, m_e1_smooth);   % "магнитный курс" в горизонтальной плоскости (ноль - произвольный, не север)
heading_mag_unwrapped = unwrap(heading_mag);
heading_mag_deg = mod(rad2deg(heading_mag_unwrapped), 360);

% производная магнитного курса - скорость поворота по магнитометру,
% для сравнения с гироскопом (gradient - численная производная по
% неравномерной сетке времени, встроенная функция MATLAB)
rate_mag = gradient(heading_mag_unwrapped, t_mag);
wz_at_mag = interp1(t_B, wz_B, t_mag, 'linear', 'extrap');

Rmat = corrcoef(rate_mag, wz_at_mag);
fprintf('Корреляция скорости поворота (магнитометр) и гироскопа: r = %.3f\n', Rmat(1,2));

figure('Name', 'B - магнитометр как датчик поворота');

subplot(3,1,1);
plot(t_mag, heading_mag_deg, 'm-'); hold on;
plot(t_good, dir_good, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
grid on; xlabel('время, с'); ylabel('курс, °');
legend('магнитный курс (произвольный ноль)', 'курс по GPS', 'Location', 'best');
title('Абсолютный магнитный курс vs GPS - должен заметно врать');

subplot(3,1,2);
plot(t_mag, rad2deg(rate_mag), 'm-'); hold on;
plot(t_mag, rad2deg(wz_at_mag), 'b-');
grid on; xlabel('время, с'); ylabel('скорость поворота, °/с');
legend('по магнитометру', 'по гироскопу', 'Location', 'best');
title(sprintf('Скорость поворота: магнитометр vs гироскоп (r=%.2f)', Rmat(1,2)));

subplot(3,1,3);
plot(t_mag, m_norm, 'k');
grid on; xlabel('время, с'); ylabel('|B|, мкТл');
title('Модуль магнитного поля - гуляет из-за кузова машины, электроники и т.п.');

saveas(gcf, fullfile('..', 'figs', 'step4_magnetometer.png'));

%% 6) Таблица поворотов: сравниваем угол поворота по трём источникам
% Находим повороты автоматически: участки, где |wz| дольше заданного
% времени держится выше порога (та же идея фильтрации дребезга, что
% в соседнем Simulink-проекте с дорожной картой - короткие "иголки" от
% дрожания руля/дороги не считаем настоящим поворотом).
thresh = 0.2;     % рад/с (~11.5 град/с)
min_dur = 0.3;    % с

is_turn = abs(wz_B_smooth) > thresh;
edges_d = diff([0; is_turn; 0]);
starts = find(edges_d == 1);
ends   = find(edges_d == -1) - 1;

fprintf('\nТаблица поворотов (гироскоп / магнитометр / GPS), градусы:\n');
fprintf('%8s %8s %10s %10s %10s\n', 't_нач', 't_кон', 'd_гиро', 'd_магн', 'd_GPS');

for k = 1:numel(starts)
    dur = t_B(ends(k)) - t_B(starts(k));
    if dur < min_dur
        continue   % слишком короткий всплеск - не настоящий поворот
    end
    ts = t_B(starts(k)); te = t_B(ends(k));

    d_gyro = rad2deg(psi_gyro(ends(k)) - psi_gyro(starts(k)));

    hm_s = interp1(t_mag, heading_mag_unwrapped, ts, 'linear', 'extrap');
    hm_e = interp1(t_mag, heading_mag_unwrapped, te, 'linear', 'extrap');
    d_mag = rad2deg(hm_e - hm_s);

    dg_s = interp1(t_good, dir_good, ts, 'linear', NaN);
    dg_e = interp1(t_good, dir_good, te, 'linear', NaN);
    d_gps = mod(dg_e - dg_s + 180, 360) - 180;   % кратчайшая разница углов (учитывая переход через 0/360)

    fprintf('%8.1f %8.1f %10.1f %10.1f %10.1f\n', ts, te, d_gyro, d_mag, d_gps);
end
