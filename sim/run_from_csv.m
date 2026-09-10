%% run_from_csv.m
% Same GPS-free navigation pipeline as run_v0.m, but the ground-truth
% trajectory is loaded from a recorded CSV file (e.g. exported from a
% separately-built game) instead of a mouse-drawn route. Kept as its own
% script - run_v0.m is untouched, so the original manual-route build is
% still there whenever you want it, just run run_v0 instead of this file.
%
% Expected CSV columns (header row required): t,x,y,psi,v,a,w - see
% truth/load_trajectory_from_csv.m for the exact format, and
% data/game_run_20260910T081331784Z.csv for a real example file.
%
% Run this file directly in MATLAB R2024a (Run button, or F5, or type
% "run_from_csv" with sim/ as the current folder).
%
% Русское резюме: тот же пайплайн, что и run_v0.m, но эталонная
% траектория не рисуется мышью, а загружается из CSV-файла (например,
% выгруженного из отдельно сделанной игры). run_v0.m при этом никак не
% меняется - запуск с ручным вводом маршрута мышью остаётся как был.

close all; clear; clc;   % закрываем все окна графиков, чистим рабочее пространство и командное окно

here = fileparts(mfilename('fullpath'));  % путь к папке, где лежит этот файл (sim/)
addpath(here);                              % добавляем sim/ в путь MATLAB
addpath(fullfile(here, '..', 'map'));       % добавляем map/ в путь MATLAB
addpath(fullfile(here, '..', 'truth'));     % добавляем truth/ в путь MATLAB

%% 1) Build the road map
% Uses the 3x3 map (not the 5x5 one from run_v0.m/generate_map.m) - it's
% the one meant to be rebuilt by hand in an external game (see
% map/MAP_3X3_INFO.md), so CSV recordings actually line up with real
% roads here instead of a map the game never saw.
map = generate_map_3x3();   % строим карту 3x3, ту же, что описана для игры в map/MAP_3X3_INFO.md

%% 2) Ground-truth trajectory: load it from a recorded CSV
% Point this at your own file if it's not the example one committed to
% data/ - see truth/load_trajectory_from_csv.m for the required columns.
csv_path = fullfile(here, '..', 'data', 'game_run_20260910T083439851Z.csv');
% ^ путь к CSV-файлу с траекторией - поменяйте на свой при необходимости
truth = load_trajectory_from_csv(csv_path);   % загружаем эталонную траекторию из CSV вместо рисования мышью
dt = truth.dt;   % шаг времени - берём из самого файла (а не задаём вручную, как для маршрута мышью)

%% 3) (Re)build the Simulink model every run
% Always rebuilt from build_model.m rather than reused from disk - same
% reasoning as run_v0.m.
mdl = 'gps_free_nav_v0';   % имя Simulink-модели (то же самое, что и в run_v0.m)
mdl_path = fullfile(here, '..', 'models', 'gps_free_nav_v0.slx');  % путь, куда сохранять .slx файл

% If a model with this name is already loaded (e.g. from a previous
% run_v0 run in this MATLAB session), Simulink refuses to load/overwrite
% it - close it first.
if bdIsLoaded(mdl)          % модель с таким именем уже загружена в память?
    close_system(mdl, 0);    % закрываем её без сохранения, чтобы не мешала пересборке
end

params = sensor_params();          % параметры модели ошибок датчика

% Real/game-recorded steering has near-constant small yaw-rate wobble
% even while driving essentially straight (measured on an actual
% recording: ~30% of samples exceeded the default 0.35 rad/s "turn"
% threshold) - with run_v0.m's defaults (capture_radius=inf,
% debounce_time=0.05) almost every one of those wobbles used to get
% mistaken for a real turn and yank the estimate to whatever map node
% was nearest, instead of the genuine corners. A longer debounce (the
% rate has to stay past the threshold for longer to count) and a finite
% capture radius (skip the correction instead of snapping to a distant,
% almost certainly wrong node) both measurably help - verified in Python
% against real recordings plus injected sensor noise before setting
% these. See sim/blocks/ins_mechanization_block.m and sim/build_model.m
% for the full explanation.
% Русское резюме: у реальной записи из игры угол всё время немного
% "дрожит", даже когда машина едет прямо - со старыми настройками
% (radius=inf, debounce=0.05) это дрожание постоянно принималось за
% настоящий поворот и дёргало оценку к ближайшему узлу. Здесь задаём
% другие значения (конечный радиус захвата + больший debounce),
% проверенные на реальных записях с добавленным шумом датчика.
mech_params.capture_radius = 30;   % м - радиус захвата для магнита к перекрёстку
mech_params.debounce_time  = 0.3;  % с - сколько нужно продержаться за порогом, чтобы поворот засчитался

build_model(dt, params, mdl_path, mech_params); % программно строим Simulink-модель заново, с шагом dt из CSV-файла

%% 4) Provide the model's inputs (it reads these base-workspace variables)
a_true_ts = [truth.t, truth.a_true]; %#ok<NASGU> (used by the From Workspace block)
% ^ истинное ускорение во времени - вход для блока From Workspace в модели
w_true_ts = [truth.t, truth.w_true]; %#ok<NASGU>
% ^ истинная угловая скорость во времени - вход для блока From Workspace
map_nodes = map.nodes;               %#ok<NASGU>
% ^ узлы карты - читаются константными блоками внутри модели
map_edges = map.edges;               %#ok<NASGU>
% ^ дороги карты - читаются константными блоками внутри модели

ins_x0   = truth.x(1);   %#ok<NASGU>
% ^ начальная координата x для блока dead reckoning (берём из загруженного файла)
ins_y0   = truth.y(1);   %#ok<NASGU>
% ^ начальная координата y
ins_psi0 = truth.psi(1); %#ok<NASGU>
% ^ начальный курс
ins_v0   = truth.v(1);   %#ok<NASGU>
% ^ начальная скорость

set_param(mdl, 'StopTime', num2str(truth.t(end)));   % задаём длительность симуляции = длительности записи

%% 5) Simulate
simOut = sim(mdl);   % запускаем симуляцию модели, результаты забираем явно (надёжнее для разных версий MATLAB)
% Produces, packaged in simOut (one per "To Workspace" block):
%   meas_log     - [a_meas, w_meas]        noisy sensor output
%   est_log      - [x,y,psi,v] estimate    raw dead reckoning (drifts!)
%   matched_log  - [xm, ym]                after snapping to the road map
%   mismatch_log - distance to the matched road segment
%   edge_log     - index of the matched road segment

%% 6) Compare true vs. dead-reckoned vs. map-matched trajectories
est     = simOut.get('est_log').Data;      % columns: x_est, y_est, psi_est, v_est
% ^ оценка dead reckoning (красная траектория): x, y, курс, скорость
matched = simOut.get('matched_log').Data;  % columns: xm, ym
% ^ результат привязки к карте (синяя траектория): x, y
meas    = simOut.get('meas_log').Data;     % columns: a_meas, w_meas
% ^ измеренные (зашумлённые) ускорение и угловая скорость

figure('Name', 'GPS-free navigation (from CSV) - trajectory');   % новое окно с графиком траекторий
hold on; axis equal; grid on;                              % не стирать предыдущие линии; равный масштаб; сетка
plot_map(map);                                              % рисуем карту дорог
plot(truth.x, truth.y, 'k-', 'LineWidth', 2, 'DisplayName', 'Truth (from CSV)');
% ^ чёрная линия - истинная траектория из загруженного файла
plot(est(:,1), est(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning (drifts)');
% ^ красная пунктирная - чистое счисление пути (уплывает от истины)
plot(matched(:,1), matched(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched');
% ^ синяя линия - после привязки к карте
legend('Location', 'best');    % легенда графика
xlabel('x, m'); ylabel('y, m'); % подписи осей
title('True (CSV) vs. dead-reckoned vs. map-matched trajectory'); % заголовок графика

figure('Name', 'GPS-free navigation (from CSV) - error');   % новое окно с графиком ошибки
err_dr = hypot(est(:,1) - truth.x, est(:,2) - truth.y);
% ^ ошибка dead reckoning в каждый момент времени (расстояние до истины)
err_mm = hypot(matched(:,1) - truth.x, matched(:,2) - truth.y);
% ^ ошибка после привязки к карте в каждый момент времени
hold on; grid on;   % не стирать предыдущие линии; сетка
plot(truth.t, err_dr, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning error');
% ^ красная линия - ошибка dead reckoning во времени
plot(truth.t, err_mm, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched error');
% ^ синяя линия - ошибка после привязки к карте во времени
legend('Location', 'best');
xlabel('t, s'); ylabel('position error, m');
title('Position error vs. time (CSV run)');

fprintf('Final dead reckoning error:  %.2f m\n', err_dr(end));   % печатаем итоговую ошибку dead reckoning
fprintf('Final map-matched error:     %.2f m\n', err_mm(end));   % печатаем итоговую ошибку после привязки к карте

%% 7) Diagnostics: does the gyro actually "see" each turn, and when does
% the estimated heading start to diverge from the true heading? Same
% purpose as in run_v0.m.
figure('Name', 'GPS-free navigation (from CSV) - diagnostics');   % новое окно с диагностическими графиками

subplot(2,1,1);   % верхний график: гироскоп - истина против измерения
plot(truth.t, rad2deg(truth.w_true), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True yaw rate');
% ^ чёрная линия - истинная угловая скорость (в градусах в секунду)
hold on; grid on;
plot(truth.t, rad2deg(meas(:,2)), 'm-', 'DisplayName', 'Measured (noisy) yaw rate');
% ^ фиолетовая линия - измеренная (зашумлённая) угловая скорость
legend('Location', 'best');
xlabel('t, s'); ylabel('yaw rate, deg/s');
title('Does the gyro see each turn? (spikes should line up with truth)');

subplot(2,1,2);   % нижний график: курс во времени - истина против оценки
plot(truth.t, rad2deg(unwrap(truth.psi)), 'k-', 'LineWidth', 2, 'DisplayName', 'True heading');
% ^ чёрная линия - истинный курс (unwrap - чтобы не было ложных скачков на +-360)
hold on; grid on;
plot(truth.t, rad2deg(unwrap(est(:,3))), 'r--', 'DisplayName', 'Dead-reckoned heading');
% ^ красная линия - оценённый курс (dead reckoning)
legend('Location', 'best');
xlabel('t, s'); ylabel('heading (psi), deg');
title('Heading over time: look for a growing gap BEFORE each turn');
