%% step1_read_and_look.m — первый взгляд на данные (телефон B, phyphox)
%
% Параллельно phoneA/step1_read_and_look.m, но для телефона B. Отличия
% в формате файлов phyphox от Sensor Logger:
%   - заголовки столбцов - "Time (s)", "X (m/s^2)", ... то есть с
%     единицами измерения прямо в названии. Чтобы не гадать, как MATLAB
%     сократит такое название до имени переменной, читаем с опцией
%     'VariableNamingRule','preserve' - тогда имена остаются как в
%     файле, и колонку берём по оригинальному тексту: T.("X (m/s^2)");
%   - порядок столбцов - Time, X, Y, Z (а не z,y,x, как у телефона A -
%     тут такой "ловушки" нет);
%   - Accelerometer.csv у phyphox - ПОЛНОЕ ускорение, С гравитацией (в
%     отличие от Accelerometer.csv Sensor Logger, где гравитация уже
%     вычтена!). Значит, в покое модуль здесь должен быть ~9.81, а не
%     ~0. "Безгравитационная" версия - отдельный файл, Linear
%     Accelerometer.csv (см. step1_LinearAccel.m).


%РАЗОБРАТЬСЯ-------------------------------------------------------------

clear; clc; close all;

if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Акселерометр (полный, с гравитацией)
accel_file = fullfile('..', 'data', 'phone_B_phyphox', 'Accelerometer.csv');
T = readtable(accel_file, 'VariableNamingRule', 'preserve');

t_s = T.("Time (s)");
ax  = T.("X (m/s^2)");
ay  = T.("Y (m/s^2)");
az  = T.("Z (m/s^2)");

a_norm = sqrt(ax.^2 + ay.^2 + az.^2);

figure('Name', 'B - акселерометр (с гравитацией)');

subplot(3,2,1);
plot(t_s, ax, 'r', t_s, ay, 'g', t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('ускорение, м/с^2');
legend('x', 'y', 'z', 'Location', 'best');
title('Все три оси вместе');

subplot(3,2,2);
plot(t_s, ax, 'r');
grid on; xlabel('время, с'); ylabel('a_x, м/с^2');
title('Ось x');

subplot(3,2,3);
plot(t_s, ay, 'g');
grid on; xlabel('время, с'); ylabel('a_y, м/с^2');
title('Ось y');

subplot(3,2,4);
plot(t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('a_z, м/с^2');
title('Ось z');

subplot(3,2,5);
plot(t_s, a_norm, 'k');
grid on; xlabel('время, с'); ylabel('|a|, м/с^2');
title('Модуль |a| - в покое ожидаем ~9.81 (гравитация ещё внутри)');

%% 2) Гироскоп
gyro_file = fullfile('..', 'data', 'phone_B_phyphox', 'Gyroscope.csv');
G = readtable(gyro_file, 'VariableNamingRule', 'preserve');

t_gyro_s = G.("Time (s)");
wx = G.("X (rad/s)");
wy = G.("Y (rad/s)");
wz = G.("Z (rad/s)");

figure('Name', 'B - гироскоп');

subplot(2,2,1);
plot(t_gyro_s, rad2deg(wx), 'r');
grid on; xlabel('время, с'); ylabel('\omega_x, град/с');
title('Гироскоп, ось x');

subplot(2,2,2);
plot(t_gyro_s, rad2deg(wy), 'g');
grid on; xlabel('время, с'); ylabel('\omega_y, град/с');
title('Гироскоп, ось y');

subplot(2,2,3);
plot(t_gyro_s, rad2deg(wz), 'b');
grid on; xlabel('время, с'); ylabel('\omega_z, град/с');
title('Гироскоп, ось z');
% 4-я ячейка пустая - графиков всего 3

%% 3) Магнитометр
mag_file = fullfile('..', 'data', 'phone_B_phyphox', 'Magnetometer.csv');
M = readtable(mag_file, 'VariableNamingRule', 'preserve');

t_mag_s = M.("Time (s)");
mx = M.("X (µT)");
my = M.("Y (µT)");
mz = M.("Z (µT)");

m_norm = sqrt(mx.^2 + my.^2 + mz.^2);

figure('Name', 'B - магнитометр');

subplot(2,2,1);
plot(t_mag_s, mx, 'r');
grid on; xlabel('время, с'); ylabel('B_x, мкТл');
title('Магнитометр, ось x');

subplot(2,2,2);
plot(t_mag_s, my, 'g');
grid on; xlabel('время, с'); ylabel('B_y, мкТл');
title('Магнитометр, ось y');

subplot(2,2,3);
plot(t_mag_s, mz, 'b');
grid on; xlabel('время, с'); ylabel('B_z, мкТл');
title('Магнитометр, ось z');

subplot(2,2,4);
plot(t_mag_s, m_norm, 'k');
grid on; xlabel('время, с'); ylabel('|B|, мкТл');
title('Модуль магнитного поля Земли');
