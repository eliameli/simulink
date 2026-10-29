%% step1_read_and_look.m — первый взгляд на данные
%
% Пока делаем только ОДИН график: полное ускорение (с гравитацией) с
% телефона A, файл Accelerometer.csv.
% Это самый простой сенсор для начала — дальше по шагам добавим
% гироскоп, магнитометр и второй телефон.
%
% Что читаем: bins/data/phone_A_sensorlogger/Accelerometer.csv
% Что строим: 3 графика осей (x,y,z) + график модуля |a|

clear; clc; close all;

% saveas() сама не создаёт папку назначения - делаем это один раз здесь,
% чтобы не падать с "Invalid or missing path", если figs/ ещё нет
% (пустые папки не хранятся в git, поэтому после git pull её может не
% быть, даже если она есть в репозитории).
if ~exist(fullfile('..','figs'), 'dir')
    mkdir(fullfile('..','figs'));
end

%% 1) Путь к файлу
% Телефон A - приложение Sensor Logger. ВАЖНО (из задания): столбцы в
% файле идут в порядке time, seconds_elapsed, z, y, x - НЕ x,y,z. Поэтому
% ниже берём колонки по имени, а не по номеру - так не перепутаем оси,
% даже если порядок столбцов в файле неочевидный.

% ????????????????????????????

accel_file = fullfile('..', 'data', 'phone_A_sensorlogger', 'Accelerometer.csv');

%% 2) Чтение файла
% readtable сама разбирает CSV с заголовком и даёт доступ к столбцам по
% их именам (T.z, T.y, T.x, ...) - это и защищает от путаницы с порядком
% столбцов.


% ????????????????????????????

T = readtable(accel_file);

t_s = T.seconds_elapsed;   % время от начала записи, секунды
ax  = T.x;                 % ускорение по оси x телефона, м/с^2
ay  = T.y;                 % ускорение по оси y телефона, м/с^2
az  = T.z;                 % ускорение по оси z телефона, м/с^2

% Это "обычный" Accelerometer.csv Sensor Logger - по заданию он уже БЕЗ
% гравитации (несмотря на название, которое наводит на мысль об
% обратном - так помечено в задании как "подвох"). Значит, в покое все
% три оси должны быть близки к нулю, а не к 9.81.

%% 3) Графики: раскладка 3 строки x 2 столбца (5 графиков из 6 ячеек)
% subplot(rows, cols, index) - index идёт слева направо, сверху вниз:
% 1 2
% 3 4
% 5 6
figure('Name', 'А - акселерометр');

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

%% 4) График: модуль вектора ускорения
% общее ускорение - корень квадратов ускорений по осям.
% Accelerometer.csv - уже без гравитации, поэтому в покое должен быть
% близок к нулю (подтвердилось на графике).
a_norm = sqrt(ax.^2 + ay.^2 + az.^2);

subplot(3,2,5);
plot(t_s, a_norm, 'k');
grid on; xlabel('время, с'); ylabel('|a|, м/с^2');
title('Модуль |a|');



%% 5) Гироскоп — читаем файл
gyro_file = fullfile('..', 'data', 'phone_A_sensorlogger', 'Gyroscope.csv');
G = readtable(gyro_file);

t_gyro_s = G.seconds_elapsed;   % время, секунды
wx = G.x;   % угловая скорость вокруг оси x телефона, рад/с
wy = G.y;   % угловая скорость вокруг оси y телефона, рад/с
wz = G.z;   % угловая скорость вокруг оси z телефона, рад/с



%% 6) График гироскопа — по графическому правилу углы/скорости поворота
% показываем в градусах в секунду (а не в радианах) - так привычнее
% читать, хотя в коде везде храним рад/с.
figure('Name', 'А - гироскоп');

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
% 4-я ячейка (2,2,4) пустая - графиков всего 3, не 4




%% 7) Магнитометр — читаем файл
mag_file = fullfile('..', 'data', 'phone_A_sensorlogger', 'Magnetometer.csv');
M = readtable(mag_file);

t_mag_s = M.seconds_elapsed;   % время, секунды
mx = M.x;   % магнитное поле по оси x телефона, мкТл
my = M.y;   % магнитное поле по оси y телефона, мкТл
mz = M.z;   % магнитное поле по оси z телефона, мкТл


% модуль магнитного поля - длина вектора B
m_norm = sqrt(mx.^2 + my.^2 + mz.^2);

%% 8) График магнитометра
figure('Name', 'А - магнитомтетр');

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
