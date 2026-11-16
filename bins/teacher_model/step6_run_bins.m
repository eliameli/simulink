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
% ВАЖНО: берём BINS1, а не BINS. У "BINS" (как мы разобрали раньше по
% его схеме) нет внутри компенсации гравитации - в самой bins_model2.mdl
% он намеренно питается БЕЗгравитационным ускорением прямо от генератора
% траектории. Реальный акселерометр телефона всегда меряет С гравитацией
% (в покое ~9.8 м/с^2, не 0) - это ровно то, для чего сделан "BINS1" (там
% есть сумматор, вычитающий g). Отдельного файла BINS1.mdl в архиве нет -
% вырезаем эту подсистему из bins_model2.mdl один раз и сохраняем рядом.
mdl = 'BINS1_standalone';
if ~isfile([mdl '.mdl']) && ~isfile([mdl '.slx'])
    load_system('bins_model2');
    new_system(mdl);
    open_system(mdl);
    add_block('bins_model2/BINS1', [mdl '/BINS1']);
    save_system(mdl);
    close_system('bins_model2', 0);
    fprintf('Вырезал подсистему BINS1 из bins_model2.mdl в отдельный файл %s.mdl\n', mdl);
end
load_system(mdl);

% Внутри mdl - один блок, подсистема "BINS1" (5 входов, 3 выхода), без
% проводов наружу. Чтобы ExternalInput было куда подключать, добавляем
% корневые Inport/Outport и соединяем их с портами подсистемы. Каждый
% раз пересобираем заново (удаляя старые, если остались от предыдущего
% запуска) - так скрипт остаётся идемпотентным.
% сначала убираем все провода на верхнем уровне (чтобы не остались
% "висячие" соединения на портах подсистемы после удаления старых
% Inport/Outport-блоков - именно это вызывало ошибку "destination
% port already has a line connection" при повторном запуске)
root_lines = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(root_lines)
    try, delete_line(root_lines(k)); catch, end
end

old_ports = [find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport'); ...
             find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Outport')];
for k = 1:numel(old_ports)
    delete_block(old_ports{k});
end

in_names = {'In_a', 'In_w', 'In_startr', 'In_startv', 'In_startangles'};
for k = 1:numel(in_names)
    add_block('simulink/Sources/In1', [mdl '/' in_names{k}], ...
        'Position', [40, 40 + 80*(k-1), 70, 60 + 80*(k-1)], ...
        'PortDimensions', '3');   % каждый вход - вектор из 3 чисел, не скаляр
    add_line(mdl, [in_names{k} '/1'], ['BINS1/' num2str(k)], 'autorouting', 'on');
end
out_names = {'Out_re', 'Out_ve', 'Out_ang'};
for k = 1:numel(out_names)
    add_block('simulink/Sinks/Out1', [mdl '/' out_names{k}], ...
        'Position', [460, 60 + 90*(k-1), 490, 80 + 90*(k-1)]);
    add_line(mdl, ['BINS1/' num2str(k)], [out_names{k} '/1'], 'autorouting', 'on');
end
save_system(mdl);
fprintf('Пересобрал корневые Inport/Outport и сохранил %s.mdl\n', mdl);

set_param(mdl, 'LoadExternalInput', 'on', 'ExternalInput', 'u');
set_param(mdl, 'StopTime', num2str(t_run(end)));
set_param(mdl, 'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Array');
set_param(mdl, 'SaveTime', 'on', 'TimeSaveName', 'tout');
% модель старая (сохранена в формате ~2015 года) - без этого sim() вернёт
% просто массив времени вместо объекта с .get(...)
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');

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

%% 7) Сохраняем для следующего шага (наложение на карту OSM)
save('step6_run_bins_result.mat', 'gps_east', 'gps_north', ...
    'bins_east_aligned', 'bins_north_aligned', 'gps_east_at_anchor', 'gps_north_at_anchor', ...
    'lat0', 'lon0', 't_gps_good', 'err_dist');
