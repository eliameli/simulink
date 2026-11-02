%% step3_manual_peak_check.m — проверка сдвига по одному выбранному повороту
%
% Проще, чем полный перебор в step3_sync.m: сами смотрите на графики
% wz(t) (их строит step3_sync.m), находите на глаза один явный,
% однозначный поворот - отдельно у телефона A и у телефона B (часы ещё
% не синхронизированы, поэтому это разные номера секунд для одного и
% того же реального момента) - вписываете окно времени вокруг него ниже,
% и скрипт сам находит момент максимума |wz| в каждом окне. Разница этих
% моментов - оценка сдвига по одному конкретному повороту, которую можно
% сверить с результатом полного перебора по всей записи.
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/).
% Поменяйте окна ниже под другой поворот и запустите заново.

clear; clc; close all;

%% 1) Окна времени для поиска пика - СВОИ для каждого телефона
tA_start = 20;   tA_end = 200;   % телефон A, с
tB_start = 20;   tB_end = 200;   % телефон B, с - подберите под тот же реальный поворот

%% 2) Телефон A: скорость рысканья (та же логика, что в step3_sync.m)
gravA = readtable(fullfile('data', 'phone_A_sensorlogger', 'Gravity.csv'));
gyroA = readtable(fullfile('data', 'phone_A_sensorlogger', 'Gyroscope.csv'));

t_gravA = gravA.seconds_elapsed;
g_A = [gravA.x, gravA.y, gravA.z];
t_A = gyroA.seconds_elapsed;
w_A = [gyroA.x, gyroA.y, gyroA.z];

g_A_i = interp1(t_gravA, g_A, t_A, 'linear', 'extrap');
up_A = -g_A_i ./ vecnorm(g_A_i, 2, 2);
wz_A = sum(w_A .* up_A, 2);
wz_A_smooth = movmean(wz_A, 20);

%% 3) Телефон B: гравитация - оценка через скользящее среднее полного ускорения
accB = readtable(fullfile('data', 'phone_B_phyphox', 'Accelerometer.csv'), ...
    'VariableNamingRule', 'preserve');
gyroB = readtable(fullfile('data', 'phone_B_phyphox', 'Gyroscope.csv'), ...
    'VariableNamingRule', 'preserve');

t_accB = accB.("Time (s)");
a_full_B = [accB.("X (m/s^2)"), accB.("Y (m/s^2)"), accB.("Z (m/s^2)")];
g_B_est = movmean(a_full_B, 200);

t_B = gyroB.("Time (s)");
w_B = [gyroB.("X (rad/s)"), gyroB.("Y (rad/s)"), gyroB.("Z (rad/s)")];

g_B_i = interp1(t_accB, g_B_est, t_B, 'linear', 'extrap');
up_B = -g_B_i ./ vecnorm(g_B_i, 2, 2);
wz_B = sum(w_B .* up_B, 2);
wz_B_smooth = movmean(wz_B, 20);

%% 4) Максимум |wz| внутри выбранного окна - для каждого телефона
% abs() - потому что поворот может быть и влево, и вправо (знак разный),
% а нас интересует самый заметный поворот в окне, неважно в какую сторону.
maskA = t_A >= tA_start & t_A <= tA_end;
tA_sub = t_A(maskA);
wzA_sub = wz_A_smooth(maskA);
[~, iA] = max(abs(wzA_sub));
t_peak_A = tA_sub(iA);

maskB = t_B >= tB_start & t_B <= tB_end;
tB_sub = t_B(maskB);
wzB_sub = wz_B_smooth(maskB);
[~, iB] = max(abs(wzB_sub));
t_peak_B = tB_sub(iB);

shift_one_event = t_peak_B - t_peak_A;

fprintf('Пик у A: t = %.2f с\n', t_peak_A);
fprintf('Пик у B: t = %.2f с\n', t_peak_B);
fprintf('Сдвиг по этому одному повороту: %.2f с\n', shift_one_event);
fprintf('(сравните с результатом полного перебора из step3_sync.m)\n');

%% 5) График - выбранные окна с отмеченным пиком
figure('Name', 'Проверка сдвига по одному повороту');

subplot(2,1,1);
plot(tA_sub, wzA_sub, 'r'); hold on;
plot(t_peak_A, wzA_sub(iA), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
grid on; xlabel('время A, с'); ylabel('\omega_z, рад/с');
title(sprintf('Телефон A: пик при t = %.2f с', t_peak_A));

subplot(2,1,2);
plot(tB_sub, wzB_sub, 'b'); hold on;
plot(t_peak_B, wzB_sub(iB), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
grid on; xlabel('время B, с'); ylabel('\omega_z, рад/с');
title(sprintf('Телефон B: пик при t = %.2f с', t_peak_B));
