%% step1Uncalibrated.m — то же самое, но на СЫРЫХ (Uncalibrated) данных
%
% Копия step1_read_and_look.m, но вместо калиброванных файлов читает
% Uncalibrated-версии (акселерометр, гироскоп, магнитометр) телефона A -
% чтобы сравнить, что меняет калибровка iOS. Акселерометр тут в других
% единицах (g, не м/с^2) и с гравитацией - см. комментарий ниже.
%
% Как запускать: открыть в MATLAB, нажать Run (текущая папка - bins/)

clear; clc; close all;

% saveas() сама не создаёт папку назначения - на случай, если её ещё нет
% (пустые папки не хранятся в git).
if ~exist('figs', 'dir')
    mkdir('figs');
end

%% 1) Путь к файлу
% Телефон A - приложение Sensor Logger. ВАЖНО (из задания): столбцы в
% файле идут в порядке time, seconds_elapsed, z, y, x - НЕ x,y,z. Поэтому
% ниже берём колонки по имени, а не по номеру - так не перепутаем оси,
% даже если порядок столбцов в файле неочевидный.
accel_file = fullfile('data', 'phone_A_sensorlogger', 'AccelerometerUncalibrated.csv');

%% 2) Чтение файла
% readtable сама разбирает CSV с заголовком и даёт доступ к столбцам по
% их именам (T.z, T.y, T.x, ...) - это и защищает от путаницы с порядком
% столбцов.
T = readtable(accel_file);

t_s = T.seconds_elapsed;   % время от начала записи, секунды
ax  = T.x;                 % ускорение по оси x телефона, в единицах g (!)
ay  = T.y;                 % ускорение по оси y телефона, в единицах g (!)
az  = T.z;                 % ускорение по оси z телефона, в единицах g (!)

% AccelerometerUncalibrated.csv - это СЫРЫЕ показания, в отличие от
% Accelerometer.csv здесь ДВА отличия сразу:
%   1) единицы измерения - не м/с^2, а g (1 g = 9.80665 м/с^2);
%   2) гравитация НЕ вычтена.
% Телефон лежит экраном вверх => ось z "смотрит" примерно вдоль
% гравитации, поэтому в покое ожидаем az ~ -1 (а не 0 и не -9.81),
% и модуль |a| в покое ~ 1 (а не 0 и не 9.81). Подтвердилось на графике.

%% 3) Графики: раскладка 3 строки x 2 столбца (5 графиков из 6 ячеек)
figure('Name', 'А - акселерометр Uncalibrated (единицы - g, с гравитацией)');

subplot(3,2,1);
plot(t_s, ax, 'r', t_s, ay, 'g', t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('ускорение, g');
legend('x', 'y', 'z', 'Location', 'best');
title('Все три оси вместе');

subplot(3,2,2);
plot(t_s, ax, 'r');
grid on; xlabel('время, с'); ylabel('a_x, g');
title('Ось x');

subplot(3,2,3);
plot(t_s, ay, 'g');
grid on; xlabel('время, с'); ylabel('a_y, g');
title('Ось y');

subplot(3,2,4);
plot(t_s, az, 'b');
grid on; xlabel('время, с'); ylabel('a_z, g');
title('Ось z');

%% 4) График: модуль вектора ускорения (в единицах g)
a_norm = sqrt(ax.^2 + ay.^2 + az.^2);

subplot(3,2,5);
plot(t_s, a_norm, 'k');
grid on; xlabel('время, с'); ylabel('|a|, g');
title('Модуль |a| - в покое ожидаем ~1, не ~9.81');



%% 5) Гироскоп — читаем файл
gyro_file = fullfile('data', 'phone_A_sensorlogger', 'GyroscopeUncalibrated.csv');
G = readtable(gyro_file);

t_gyro_s = G.seconds_elapsed;   % время, секунды
wx = G.x;   % угловая скорость вокруг оси x телефона, рад/с
wy = G.y;   % угловая скорость вокруг оси y телефона, рад/с
wz = G.z;   % угловая скорость вокруг оси z телефона, рад/с



%% 6) График гироскопа — по графическому правилу углы/скорости поворота
% показываем в градусах в секунду (а не в радианах) - так привычнее
% читать, хотя в коде везде храним рад/с.
figure('Name', 'А - гироскоп Uncalibrated');

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
mag_file = fullfile('data', 'phone_A_sensorlogger', 'MagnetometerUncalibrated.csv');
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
% MagnetometerUncalibrated.csv - те же мкТл, что и в калиброванном
% файле, но тут добавлен большой постоянный сдвиг (hard iron, сотни
% мкТл) от самого телефона - его iOS и пытается вычесть в Magnetometer.csv.
figure('Name', 'А - магнитометр Uncalibrated');

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
title('Модуль - ожидаем сотни мкТл (большой сдвиг от телефона)');

saveas(gcf, fullfile('figs', 'step1_phoneA_mag_uncal.png'));
