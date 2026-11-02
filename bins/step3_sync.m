%% step3_sync.m — синхронизация телефонов A и B, задержка GPS
%
% Два телефона пишут время каждый от своего старта записи - эти
% временные шкалы никак не совпадают сами по себе. Здесь ищем сдвиг
% между ними по физике: оба телефона в одной машине чувствуют одни и те
% же реальные повороты, значит их "скорость рысканья" (вращение вокруг
% вертикали) должна быть одним и тем же сигналом, просто сдвинутым по
% времени - сдвиг, при котором сигналы совпадают лучше всего (пик
% корреляции), и есть искомая рассинхронизация часов.
%
% Отдельно оцениваем задержку GPS-чипа телефона B относительно его же
% инерциальных данных - тем же способом (корреляция при переборе
% сдвига), но внутри одного телефона.
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/)

clear; clc; close all;

if ~exist('figs', 'dir')
    mkdir('figs');
end

%% 1) Прикидка сдвига по часам телефонов (ТОЛЬКО чтобы сузить перебор)
% Часы двух независимых устройств могут слегка врать - поэтому это не
% ответ, а просто центр, вокруг которого ищем точный сдвиг по сигналам.
metaA = readtable(fullfile('data', 'phone_A_sensorlogger', 'Metadata.csv'), ...
    'VariableNamingRule', 'preserve');
startA_s = metaA.("recording epoch time") / 1000;   % было в мс -> переводим в секунды

metaB = readtable(fullfile('data', 'phone_B_phyphox', 'meta', 'time.csv'), ...
    'VariableNamingRule', 'preserve');
is_start = strcmp(metaB.event, 'START');
startB_s = metaB.("system time")(is_start);          % уже в секундах (unix-время)

shift_clock = startA_s - startB_s;   % секунд: время_B = время_A + shift_clock
fprintf('Сдвиг по часам (прикидка): %.3f с\n', shift_clock);

%% 2) Телефон A: скорость рысканья wz_A = проекция угловой скорости на "вверх"
% "Вверх" = -(гравитация)/|гравитация| - у Sensor Logger гравитация уже
% прямо дана (Gravity.csv), поэтому для A ничего оценивать не нужно.
gravA = readtable(fullfile('data', 'phone_A_sensorlogger', 'Gravity.csv'));
gyroA = readtable(fullfile('data', 'phone_A_sensorlogger', 'Gyroscope.csv'));

t_gravA = gravA.seconds_elapsed;
g_A = [gravA.x, gravA.y, gravA.z];

t_A = gyroA.seconds_elapsed;
w_A = [gyroA.x, gyroA.y, gyroA.z];

% гравитация и гироскоп пишутся отдельными потоками - их метки времени
% не совпадают один-в-один, интерполируем гравитацию на моменты гироскопа
g_A_i = interp1(t_gravA, g_A, t_A, 'linear', 'extrap');
up_A = -g_A_i ./ vecnorm(g_A_i, 2, 2);

wz_A = sum(w_A .* up_A, 2);          % скорость рысканья, рад/с
wz_A_smooth = movmean(wz_A, 20);     % сглаживание (~0.2 с при ~100 Гц)

%% 3) Телефон B: гравитация тут отдельным файлом не дана - оцениваем её
% сами как медленно меняющуюся (низкочастотную) часть полного
% ускорения: машина разгоняется/тормозит/поворачивает быстро, а
% гравитация в осях телефона меняется только если телефон физически
% переориентировали - то есть почти не меняется за секунды.
accB = readtable(fullfile('data', 'phone_B_phyphox', 'Accelerometer.csv'), ...
    'VariableNamingRule', 'preserve');
gyroB = readtable(fullfile('data', 'phone_B_phyphox', 'Gyroscope.csv'), ...
    'VariableNamingRule', 'preserve');

t_accB = accB.("Time (s)");
a_full_B = [accB.("X (m/s^2)"), accB.("Y (m/s^2)"), accB.("Z (m/s^2)")];
g_B_est = movmean(a_full_B, 200);   % скользящее среднее, ~2 с при ~100 Гц - оценка гравитации

t_B = gyroB.("Time (s)");
w_B = [gyroB.("X (rad/s)"), gyroB.("Y (rad/s)"), gyroB.("Z (rad/s)")];

g_B_i = interp1(t_accB, g_B_est, t_B, 'linear', 'extrap');
up_B = -g_B_i ./ vecnorm(g_B_i, 2, 2);

wz_B = sum(w_B .* up_B, 2);
wz_B_smooth = movmean(wz_B, 20);

%% 4) Перебор сдвига A->B с шагом 0.01 с, ищем максимум корреляции
% Гипотеза: время_B = время_A + shift. Для каждого кандидата shift
% пересчитываем wz_B на моменты времени телефона A (через интерполяцию)
% и смотрим, насколько хорошо он совпадает с wz_A.
shifts = (shift_clock - 5) : 0.01 : (shift_clock + 5);
r_list = nan(size(shifts));

for i = 1:numel(shifts)
    sh = shifts(i);
    wzB_q = interp1(t_B, wz_B_smooth, t_A + sh, 'linear', NaN);
    valid = ~isnan(wzB_q);
    if nnz(valid) < 100
        continue
    end
    Rmat = corrcoef(wz_A_smooth(valid), wzB_q(valid));
    r_list(i) = Rmat(1,2);
end

[best_r, idx] = max(r_list);
best_shift = shifts(idx);
fprintf('Сдвиг по корреляции wz: %.2f с (корреляция r=%.3f)\n', best_shift, best_r);

%% 5) Графики: корреляция от сдвига, и wz до/после выравнивания
figure('Name', 'Синхронизация A и B');

subplot(3,1,1);
plot(shifts, r_list, 'b');
grid on; xlabel('сдвиг, с'); ylabel('r');
title(sprintf('Корреляция wz(A) и wz(B) от сдвига - пик при %.2f с', best_shift));

subplot(3,1,2);
plot(t_A, wz_A_smooth, 'r'); hold on;
plot(t_B, wz_B_smooth, 'b');
grid on; xlabel('время каждого телефона (своё, без сдвига), с'); ylabel('\omega_z, рад/с');
legend('A', 'B', 'Location', 'best');
title('ДО выравнивания - сигналы на своих, несовпадающих, шкалах времени');

subplot(3,1,3);
plot(t_A, wz_A_smooth, 'r'); hold on;
plot(t_B - best_shift, wz_B_smooth, 'b');
grid on; xlabel('время телефона A, с'); ylabel('\omega_z, рад/с');
legend('A', 'B (сдвинут)', 'Location', 'best');
title('ПОСЛЕ выравнивания - должны совпасть по пикам поворотов');

saveas(gcf, fullfile('figs', 'step3_sync_AB.png'));

%% 6) Задержка GPS телефона B относительно его собственных инерциальных данных
% Идея та же: GPS-чип считает координаты не мгновенно, поэтому скорость
% по GPS "отстаёт" от реального ускорения машины на какое-то время.
% За "реальное ускорение" тут берём модуль ГОРИЗОНТАЛЬНОЙ составляющей
% ускорения без гравитации (ось машины вперёд/назад ещё не найдена -
% это шаг 4, пока используем временную замену, как предложено в задании).
linB = readtable(fullfile('data', 'phone_B_phyphox', 'Linear Accelerometer.csv'), ...
    'VariableNamingRule', 'preserve');
t_linB = linB.("Time (s)");
a_lin_B = [linB.("X (m/s^2)"), linB.("Y (m/s^2)"), linB.("Z (m/s^2)")];

g_B_at_lin = interp1(t_accB, g_B_est, t_linB, 'linear', 'extrap');
up_B_at_lin = -g_B_at_lin ./ vecnorm(g_B_at_lin, 2, 2);

a_vert = sum(a_lin_B .* up_B_at_lin, 2) .* up_B_at_lin;   % вертикальная составляющая (вектор)
a_horiz = a_lin_B - a_vert;                                % горизонтальная составляющая (вектор)
a_horiz_mag = vecnorm(a_horiz, 2, 2);                      % её модуль

% GPS: берём только "хорошие" точки (критерий из задания), считаем dv/dt
locB = readtable(fullfile('data', 'phone_B_phyphox', 'Location.csv'), ...
    'VariableNamingRule', 'preserve');
t_gps = locB.("Time (s)");
hacc_gps = locB.("Horizontal Accuracy (m)");
v_gps_raw = locB.("Velocity (m/s)");
if iscell(v_gps_raw)
    v_gps = str2double(v_gps_raw);
else
    v_gps = v_gps_raw;
end

good = hacc_gps < 20 & ~isnan(v_gps);
t_good = t_gps(good);
v_good = v_gps(good);

dt_gps = diff(t_good);
dv_gps = diff(v_good);
keep = dt_gps < 3;                 % отбрасываем большие разрывы в "хороших" точках
dvdt = dv_gps(keep) ./ dt_gps(keep);
t_mid = (t_good(1:end-1) + t_good(2:end)) / 2;
t_mid = t_mid(keep);

delays = -5:0.1:5;
r_gps = nan(size(delays));
for i = 1:numel(delays)
    d = delays(i);
    a_q = interp1(t_linB, a_horiz_mag, t_mid + d, 'linear', NaN);
    valid = ~isnan(a_q);
    if nnz(valid) < 10
        continue
    end
    Rmat = corrcoef(abs(dvdt(valid)), a_q(valid));   % |dv/dt| - знак ускорения GPS не различает направление
    r_gps(i) = Rmat(1,2);
end

[best_r_gps, idx_gps] = max(r_gps);
best_delay_gps = delays(idx_gps);
fprintf('Задержка GPS телефона B: %.2f с (корреляция r=%.3f)\n', best_delay_gps, best_r_gps);

figure('Name', 'Задержка GPS (телефон B)');
plot(delays, r_gps, 'k');
grid on; xlabel('задержка GPS, с'); ylabel('r');
title(sprintf('Корреляция |dv/dt| (GPS) и |a_{гориз}| (ИНС) - пик при %.2f с', best_delay_gps));

saveas(gcf, fullfile('figs', 'step3_gps_delay.png'));
