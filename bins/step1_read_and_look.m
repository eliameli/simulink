%% step1_read_and_look.m — первый взгляд на данные
%
% Пока делаем только ОДИН график: полное ускорение (с гравитацией) с
% телефона A (Sensor Logger, iPhone 17), файл Accelerometer.csv.
% Это самый простой сенсор для начала — дальше по шагам добавим
% гироскоп, магнитометр и второй телефон.
%
% Что читаем: bins/data/phone_A_sensorlogger/Accelerometer.csv
% Что строим: 3 графика осей (x,y,z) + график модуля |a|
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/)

clear; clc; close all;

% saveas() сама не создаёт папку назначения - делаем это один раз здесь,
% чтобы не падать с "Invalid or missing path", если figs/ ещё нет
% (пустые папки не хранятся в git, поэтому после git pull её может не
% быть, даже если она есть в репозитории).
if ~exist('figs', 'dir')
    mkdir('figs');
end

%% 1) Путь к файлу
% Телефон A - приложение Sensor Logger. ВАЖНО (из задания): столбцы в
% файле идут в порядке time, seconds_elapsed, z, y, x - НЕ x,y,z. Поэтому
% ниже берём колонки по имени, а не по номеру - так не перепутаем оси,
% даже если порядок столбцов в файле неочевидный.
accel_file = fullfile('data', 'phone_A_sensorlogger', 'Accelerometer.csv');

%% 2) Чтение файла
% readtable сама разбирает CSV с заголовком и даёт доступ к столбцам по
% их именам (T.z, T.y, T.x, ...) - это и защищает от путаницы с порядком
% столбцов.
T = readtable(accel_file);

t_s = T.seconds_elapsed;   % время от начала записи, секунды
ax  = T.x;                 % ускорение по оси x телефона, м/с^2
ay  = T.y;                 % ускорение по оси y телефона, м/с^2
az  = T.z;                 % ускорение по оси z телефона, м/с^2

% Это "обычный" Accelerometer.csv Sensor Logger - по заданию он уже БЕЗ
% гравитации (несмотря на название, которое наводит на мысль об
% обратном - так помечено в задании как "подвох"). Значит, в покое все
% три оси должны быть близки к нулю, а не к 9.81.

%% 3) График: три оси по отдельности
figure('Name', 'А - акселерометр');
subplot(5,1,1);
plot(t_s, ax, 'r', t_s, ay, 'g', t_s, az, 'b');
xlabel('время, с'); ylabel('ускорение, м/с^2');



subplot(5,1,2);
plot(t_s, ax, 'r');
grid on; xlabel('время, с'); ylabel('a_x, м/с^2');
title('Ось x');

subplot(5,1,3);
plot(t_s, ay, 'g');
grid on; xlabel('время, с'); ylabel('a_y, м/с^2');
title('Ось y');

subplot(5,1,4);
plot(t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('a_z, м/с^2');
title('Ось z');


grid on;
xlabel('время, с'); ylabel('ускорение, м/с^2');

title('Телефон A: Accelerometer.csv - ускорение по трём осям телефона');

%% 4) График: модуль вектора ускорения


% общее ускорение - корень квадратов ускорений по осям
% если > 9.81 в покое то система его считает

a_norm = sqrt(ax.^2 + ay.^2 + az.^2);

subplot(5,1,5);
plot(t_s, a_norm, 'k');
grid on; xlabel('время, с'); ylabel('|a|, м/с^2');
title('Модуль');



%% 5) Гироскоп — читаем файл
gyro_file = fullfile('data', 'phone_A_sensorlogger', 'Gyroscope.csv');
G = readtable(gyro_file);

t_gyro_s = G.seconds_elapsed;   % время, секунды
wx = G.x;   % угловая скорость вокруг оси x телефона, рад/с
wy = G.y;   % угловая скорость вокруг оси y телефона, рад/с
wz = G.z;   % угловая скорость вокруг оси z телефона, рад/с



%% 6) График гироскопа — по графическому правилу углы/скорости поворота
% показываем в градусах в секунду (а не в радианах) - так привычнее
% читать, хотя в коде везде храним рад/с.
figure('Name', 'А - гироскоп');

subplot(3,1,1);
plot(t_gyro_s, rad2deg(wx), 'r');
grid on; xlabel('время, с'); ylabel('\omega_x, град/с');
title('Гироскоп, ось x');

subplot(3,1,2);
plot(t_gyro_s, rad2deg(wy), 'g');
grid on; xlabel('время, с'); ylabel('\omega_y, град/с');
title('Гироскоп, ось y');

subplot(3,1,3);
plot(t_gyro_s, rad2deg(wz), 'b');
grid on; xlabel('время, с'); ylabel('\omega_z, град/с');
title('Гироскоп, ось z');




%% 7) Магнитометр — читаем файл
mag_file = fullfile('data', 'phone_A_sensorlogger', 'Magnetometer.csv');
M = readtable(mag_file);

t_mag_s = M.seconds_elapsed;   % время, секунды
mx = M.x;   % магнитное поле по оси x телефона, мкТл
my = M.y;   % магнитное поле по оси y телефона, мкТл
mz = M.z;   % магнитное поле по оси z телефона, мкТл

fprintf('Телефон A, Magnetometer.csv:\n');
fprintf('  отсчётов: %d\n', height(M));
fprintf('  медианный шаг по времени: %.4f с (т.е. ~%.0f Гц)\n', ...
    median(diff(t_mag_s)), 1/median(diff(t_mag_s)));

% модуль магнитного поля - длина вектора B
m_norm = sqrt(mx.^2 + my.^2 + mz.^2);

%% 8) График магнитометра
figure('Name', 'А - магнитомтетр');

subplot(4,1,1);
plot(t_mag_s, mx, 'r');
grid on; xlabel('время, с'); ylabel('B_x, мкТл');
title('Магнитометр, ось x');

subplot(4,1,2);
plot(t_mag_s, my, 'g');
grid on; xlabel('время, с'); ylabel('B_y, мкТл');
title('Магнитометр, ось y');

subplot(4,1,3);
plot(t_mag_s, mz, 'b');
grid on; xlabel('время, с'); ylabel('B_z, мкТл');
title('Магнитометр, ось z');

subplot(4,1,4);
plot(t_mag_s, m_norm, 'k');
grid on; xlabel('время, с'); ylabel('|B|, мкТл');
title('Модуль магнитного поля Земли');

saveas(gcf, fullfile('figs', 'step1_phoneA_mag.png'));