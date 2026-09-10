function mdl_path = build_model(dt, params, mdl_path, mech_params)
%BUILD_MODEL Programmatically build the v0 Simulink model.
%   MDL_PATH = BUILD_MODEL(DT, PARAMS, MDL_PATH) builds
%   gps_free_nav_v0.slx: a synthetic IMU sensor model feeding an RK4
%   dead-reckoning ("INS mechanization") block, followed by a
%   nearest-road-segment map-matching block, with signal logging to the
%   base workspace. Saves it to MDL_PATH and returns that path.
%
%   DT     - fixed simulation step [s] (must match the truth trajectory's
%            time step, see truth/generate_trajectory.m)
%   PARAMS - sensor error parameters, see sim/sensor_params.m. These
%            values are baked into the model at build time: if you
%            change sensor_params.m, rebuild the model (delete the .slx
%            or call build_model again) for the change to take effect.
%   MECH_PARAMS - optional struct tuning the INS Mechanization block's
%            corner-snap (see sim/blocks/ins_mechanization_block.m for
%            why this needs to differ by data source):
%              .capture_radius - max distance [m] to a map node for a
%                                 turn-triggered correction to fire.
%                                 Default: inf (run_v0.m's mouse-drawn
%                                 routes need this - a correction must
%                                 always find *some* node).
%              .debounce_time  - how long [s] the yaw rate must stay
%                                 past a threshold before a turn
%                                 start/end counts. Default: 0.05.
%            Omit this argument entirely to get both defaults (i.e.
%            run_v0.m's original behavior, unchanged). run_from_csv.m
%            passes capture_radius=30, debounce_time=0.3 instead - real
%            recorded steering has near-constant small yaw-rate wobble
%            that the short default debounce mistakes for real turns far
%            more often than a hand-drawn route ever does (measured: ~30%
%            of samples on an actual recording), and each false trigger
%            used to snap the estimate to whatever node was nearest.
%
%   At simulation time (not build time) the caller must set these
%   base-workspace variables before calling sim():
%     a_true_ts, w_true_ts - [time, value] matrices, the true forward
%                             accel / yaw rate the sensor model corrupts
%     map_nodes, map_edges - the road map (see map/generate_map.m)
%   See sim/run_v0.m for a complete example.
%
%   NOTE: this script was written and reviewed without a local MATLAB to
%   test it against (see README.md). The two MATLAB Function blocks are
%   inserted via the documented Stateflow scripting API; if that step
%   fails for your MATLAB version, open the model and paste the code
%   from sim/blocks/*.m into the corresponding (then-empty) block by
%   hand - the rest of the model (wiring, sources, sinks) is built with
%   plain, very stable block-diagram APIs and should not need that.
%
% Русское резюме: этот файл программно (кодом, а не мышкой) строит всю
% Simulink-модель - добавляет блоки и соединяет их проводами. Ниже
% почти на каждой строке краткая подпись, что она делает. MECH_PARAMS -
% необязательный параметр (радиус захвата + время подтверждения поворота
% для блока dead reckoning) - см. пояснение в
% sim/blocks/ins_mechanization_block.m, почему это нужно задавать
% по-разному для маршрута мышью и для записи из игры.

if nargin < 4 || isempty(mech_params)      % значения не переданы - используем старые по умолчанию (маршрут мышью)
    mech_params = struct();                 % пустая структура - поля ниже заполнятся значениями по умолчанию
end
if ~isfield(mech_params, 'capture_radius') || isempty(mech_params.capture_radius)
    mech_params.capture_radius = inf;        % по умолчанию - без ограничения (как было изначально)
end
if ~isfield(mech_params, 'debounce_time') || isempty(mech_params.debounce_time)
    mech_params.debounce_time = 0.05;        % по умолчанию - как было изначально
end

mdl = 'gps_free_nav_v0';   % имя модели (используется и как имя файла, и как имя системы в Simulink)

if bdIsLoaded(mdl)          % модель с таким именем уже загружена в памяти MATLAB?
    close_system(mdl, 0);    % закрываем её без сохранения - иначе новую не создать поверх старой
end
new_system(mdl);    % создаём новую пустую Simulink-модель
open_system(mdl);   % открываем её окно

% ---- top-level sources -------------------------------------------------
add_block('simulink/Sources/From Workspace', [mdl '/a_true']);   % блок "из рабочего пространства" для истинного ускорения
set_param([mdl '/a_true'], 'VariableName', 'a_true_ts', ...
    'SampleTime', num2str(dt), 'Position', pos(0, 0, 90, 30));
% ^ настраиваем: откуда брать данные (переменная a_true_ts), с каким шагом времени, где на схеме рисовать блок

add_block('simulink/Sources/From Workspace', [mdl '/w_true']);   % блок "из рабочего пространства" для истинной угл. скорости
set_param([mdl '/w_true'], 'VariableName', 'w_true_ts', ...
    'SampleTime', num2str(dt), 'Position', pos(0, 1, 90, 30));
% ^ то же самое, но для переменной w_true_ts

% ---- Sensor Model subsystem --------------------------------------------
sensPath = [mdl '/Sensor Model'];         % путь к подсистеме "модель датчика"
build_sensor_subsystem(sensPath, dt, params);   % строим её содержимое (см. функцию ниже)
set_param(sensPath, 'Position', pos(1, 0.5, 140, 90));   % задаём положение подсистемы на схеме

add_line(mdl, 'a_true/1', 'Sensor Model/1', 'autorouting', 'on');   % провод: истинное ускорение -> вход 1 модели датчика
add_line(mdl, 'w_true/1', 'Sensor Model/2', 'autorouting', 'on');   % провод: истинная угл. скорость -> вход 2 модели датчика

% ---- Ts and initial-pose constants (fed into the mechanization block) --
add_block('simulink/Sources/Constant', [mdl '/Ts']);   % константный блок - шаг времени
set_param([mdl '/Ts'], 'Value', sprintf('%.10g', dt), ...
    'Position', pos(2, 2.3, 60, 30));
% ^ значение константы = dt (число, "зашито" при сборке модели)

% Dead reckoning always needs a known starting pose. The route (and
% hence the start point) is chosen interactively per run (see
% map/pick_route.m), so it can't be a literal inside the block - these
% four constants read it from the base workspace instead (run_v0.m sets
% them from the ground truth's first sample: ins_x0/ins_y0/ins_psi0/ins_v0).
% Русское резюме: начальная точка каждый раз разная (маршрут рисуется
% заново), поэтому не "зашиваем" число в блок, а читаем из рабочего
% пространства (run_v0.m задаёт переменные ins_x0/ins_y0/ins_psi0/ins_v0).
add_block('simulink/Sources/Constant', [mdl '/x0']);    % константа - начальная координата x
set_param([mdl '/x0'], 'Value', 'ins_x0', 'Position', pos(2, 2.7, 60, 30));   % берём значение из переменной ins_x0
add_block('simulink/Sources/Constant', [mdl '/y0']);    % константа - начальная координата y
set_param([mdl '/y0'], 'Value', 'ins_y0', 'Position', pos(2, 3.1, 60, 30));   % берём значение из переменной ins_y0
add_block('simulink/Sources/Constant', [mdl '/psi0']);  % константа - начальный курс
set_param([mdl '/psi0'], 'Value', 'ins_psi0', 'Position', pos(2, 3.5, 60, 30));   % берём значение из переменной ins_psi0
add_block('simulink/Sources/Constant', [mdl '/v0']);    % константа - начальная скорость
set_param([mdl '/v0'], 'Value', 'ins_v0', 'Position', pos(2, 3.9, 60, 30));   % берём значение из переменной ins_v0

% Corner-snap tuning (capture_radius, debounce_time) - baked in as
% literal numbers at build time (like Ts), not read from the base
% workspace, since they're a fixed choice for the whole run rather than
% something that changes with the route. See MECH_PARAMS above for why
% run_v0.m and run_from_csv.m pass different values here.
% Русское резюме: радиус захвата и время подтверждения поворота -
% зашиваем числом при сборке (как Ts), а не читаем из workspace, они
% одинаковы весь прогон. run_v0.m и run_from_csv.m передают сюда разные
% значения (см. MECH_PARAMS выше).
add_block('simulink/Sources/Constant', [mdl '/Capture Radius']);   % константа - радиус захвата для магнита к перекрёстку
set_param([mdl '/Capture Radius'], 'Value', sprintf('%.10g', mech_params.capture_radius), ...
    'Position', pos(2, 4.3, 60, 30));
add_block('simulink/Sources/Constant', [mdl '/Debounce Time']);   % константа - время подтверждения поворота
set_param([mdl '/Debounce Time'], 'Value', sprintf('%.10g', mech_params.debounce_time), ...
    'Position', pos(2, 4.7, 60, 30));

% ---- INS Mechanization (RK4) MATLAB Function block ---------------------
mechPath = [mdl '/INS Mechanization (RK4)'];   % путь к блоку dead reckoning
add_block('simulink/User-Defined Functions/MATLAB Function', mechPath);   % добавляем пустой блок "MATLAB Function"
set_param(mechPath, 'Position', pos(2.3, 0.5, 160, 100));   % положение блока на схеме
% This block uses a persistent variable to hold its running pose
% estimate, which Simulink does not allow on a block that inherits a
% continuous sample time. Every signal feeding it is discrete at dt
% (the Sensor Model subsystem's bias random walk uses a
% Discrete-Time Integrator, not a continuous one, for exactly this
% reason), so with no continuous states anywhere upstream this block's
% default inherited sample time correctly resolves to the single
% discrete rate dt.
set_chart_script(mechPath, fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'blocks', 'ins_mechanization_block.m')));
% ^ вставляем код (текст файла ins_mechanization_block.m) внутрь блока

add_line(mdl, 'Sensor Model/1', 'INS Mechanization (RK4)/1', 'autorouting', 'on');   % провод: измеренное ускорение -> вход 1
add_line(mdl, 'Sensor Model/2', 'INS Mechanization (RK4)/2', 'autorouting', 'on');   % провод: измеренная угл. скорость -> вход 2
add_line(mdl, 'Ts/1', 'INS Mechanization (RK4)/3', 'autorouting', 'on');             % провод: шаг времени -> вход 3
add_line(mdl, 'x0/1', 'INS Mechanization (RK4)/4', 'autorouting', 'on');             % провод: начальный x -> вход 4
add_line(mdl, 'y0/1', 'INS Mechanization (RK4)/5', 'autorouting', 'on');             % провод: начальный y -> вход 5
add_line(mdl, 'psi0/1', 'INS Mechanization (RK4)/6', 'autorouting', 'on');           % провод: начальный курс -> вход 6
add_line(mdl, 'v0/1', 'INS Mechanization (RK4)/7', 'autorouting', 'on');             % провод: начальная скорость -> вход 7
% (входы 8, 9 - карта, подключаются ниже, после блоков Map Nodes/Map Edges)
% (входы 10, 11 - радиус захвата и debounce - подключаются здесь, они уже готовы)
add_line(mdl, 'Capture Radius/1', 'INS Mechanization (RK4)/10', 'autorouting', 'on'); % провод: радиус захвата -> вход 10
add_line(mdl, 'Debounce Time/1', 'INS Mechanization (RK4)/11', 'autorouting', 'on');  % провод: время подтверждения поворота -> вход 11

% ---- Map data constants -------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/Map Nodes']);   % константа - список узлов карты
set_param([mdl '/Map Nodes'], 'Value', 'map_nodes', 'Position', pos(3.6, 2, 90, 30));   % значение - переменная map_nodes

add_block('simulink/Sources/Constant', [mdl '/Map Edges']);   % константа - список дорог карты
set_param([mdl '/Map Edges'], 'Value', 'map_edges', 'Position', pos(3.6, 2.6, 90, 30));   % значение - переменная map_edges

% INS Mechanization also needs the map (nodes + edges) for its
% debounced corner-snap correction (see
% sim/blocks/ins_mechanization_block.m) - wired here, after "Map
% Nodes"/"Map Edges" exist, rather than up in that block's section.
% Русское резюме: блоку dead reckoning тоже нужна карта (для привязки
% к перекрёсткам) - подключаем провода сюда, после того как блоки
% карты уже созданы выше.
add_line(mdl, 'Map Nodes/1', 'INS Mechanization (RK4)/8', 'autorouting', 'on');   % провод: узлы карты -> вход 8
add_line(mdl, 'Map Edges/1', 'INS Mechanization (RK4)/9', 'autorouting', 'on');   % провод: дороги карты -> вход 9

% ---- Map Matching MATLAB Function block ----------------------------------
% Per-sample nearest-road matching (with hysteresis) applied directly
% on top of the raw ("red") dead reckoning.
% Русское резюме: блок привязки к карте - работает на каждом шаге,
% берёт вход прямо из dead reckoning (красной траектории).
matchPath = [mdl '/Map Matching'];   % путь к блоку привязки к карте
add_block('simulink/User-Defined Functions/MATLAB Function', matchPath);   % добавляем пустой блок "MATLAB Function"
set_param(matchPath, 'Position', pos(4.3, 0.5, 160, 100));   % положение блока на схеме
set_chart_script(matchPath, fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'blocks', 'map_matching_block.m')));
% ^ вставляем код (текст файла map_matching_block.m) внутрь блока

add_line(mdl, 'INS Mechanization (RK4)/1', 'Map Matching/1', 'autorouting', 'on');   % провод: оценённый x -> вход 1
add_line(mdl, 'INS Mechanization (RK4)/2', 'Map Matching/2', 'autorouting', 'on');   % провод: оценённый y -> вход 2
add_line(mdl, 'Map Nodes/1', 'Map Matching/3', 'autorouting', 'on');                 % провод: узлы карты -> вход 3
add_line(mdl, 'Map Edges/1', 'Map Matching/4', 'autorouting', 'on');                 % провод: дороги карты -> вход 4

% ---- Logging: Mux each group of signals, then To Workspace -------------
% Русское резюме: объединяем нужные сигналы блоком Mux и записываем в
% рабочее пространство блоком To Workspace - так их потом можно
% забрать в run_v0.m для графиков.
add_block('simulink/Signal Routing/Mux', [mdl '/Mux Meas']);   % блок Mux - объединяет измеренные сигналы датчика в один пучок
set_param([mdl '/Mux Meas'], 'Inputs', '2', 'Position', pos(3.6, -0.6, 20, 60));   % 2 входа
add_line(mdl, 'Sensor Model/1', 'Mux Meas/1', 'autorouting', 'on');   % провод: измеренное ускорение -> вход 1 Mux
add_line(mdl, 'Sensor Model/2', 'Mux Meas/2', 'autorouting', 'on');   % провод: измеренная угл. скорость -> вход 2 Mux
add_block('simulink/Sinks/To Workspace', [mdl '/meas_log']);   % блок записи в рабочее пространство
set_param([mdl '/meas_log'], 'VariableName', 'meas_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(4.6, -0.6, 90, 30));
% ^ имя переменной meas_log, формат - временной ряд, без ограничения по числу точек
add_line(mdl, 'Mux Meas/1', 'meas_log/1', 'autorouting', 'on');   % провод: объединённый сигнал -> вход блока записи

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Est']);   % блок Mux - объединяет оценку dead reckoning (x,y,курс,скорость)
set_param([mdl '/Mux Est'], 'Inputs', '4', 'Position', pos(3.2, 0.5, 20, 80));   % 4 входа
add_line(mdl, 'INS Mechanization (RK4)/1', 'Mux Est/1', 'autorouting', 'on');   % провод: x -> вход 1 Mux
add_line(mdl, 'INS Mechanization (RK4)/2', 'Mux Est/2', 'autorouting', 'on');   % провод: y -> вход 2 Mux
add_line(mdl, 'INS Mechanization (RK4)/3', 'Mux Est/3', 'autorouting', 'on');   % провод: курс -> вход 3 Mux
add_line(mdl, 'INS Mechanization (RK4)/4', 'Mux Est/4', 'autorouting', 'on');   % провод: скорость -> вход 4 Mux
add_block('simulink/Sinks/To Workspace', [mdl '/est_log']);   % блок записи оценки dead reckoning
set_param([mdl '/est_log'], 'VariableName', 'est_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(3.4, 3.6, 90, 30));
% ^ имя переменной est_log
add_line(mdl, 'Mux Est/1', 'est_log/1', 'autorouting', 'on');   % провод: объединённый сигнал -> вход блока записи

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Matched']);   % блок Mux - объединяет результат привязки к карте (x,y)
set_param([mdl '/Mux Matched'], 'Inputs', '2', 'Position', pos(5.3, 0.3, 20, 60));   % 2 входа
add_line(mdl, 'Map Matching/1', 'Mux Matched/1', 'autorouting', 'on');   % провод: x после привязки -> вход 1 Mux
add_line(mdl, 'Map Matching/2', 'Mux Matched/2', 'autorouting', 'on');   % провод: y после привязки -> вход 2 Mux
add_block('simulink/Sinks/To Workspace', [mdl '/matched_log']);   % блок записи результата привязки к карте
set_param([mdl '/matched_log'], 'VariableName', 'matched_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.9, 0.3, 90, 30));
% ^ имя переменной matched_log
add_line(mdl, 'Mux Matched/1', 'matched_log/1', 'autorouting', 'on');   % провод: объединённый сигнал -> вход блока записи

add_block('simulink/Sinks/To Workspace', [mdl '/mismatch_log']);   % блок записи "расстояние до дороги"
set_param([mdl '/mismatch_log'], 'VariableName', 'mismatch_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.6, 1.3, 90, 30));
% ^ имя переменной mismatch_log
add_line(mdl, 'Map Matching/4', 'mismatch_log/1', 'autorouting', 'on');   % провод: расстояние до дороги -> вход блока записи

add_block('simulink/Sinks/To Workspace', [mdl '/edge_log']);   % блок записи "номер выбранной дороги"
set_param([mdl '/edge_log'], 'VariableName', 'edge_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.6, 2.3, 90, 30));
% ^ имя переменной edge_log
add_line(mdl, 'Map Matching/3', 'edge_log/1', 'autorouting', 'on');   % провод: номер дороги -> вход блока записи

% ---- Solver: fixed-step RK4 (ode4), NOT Euler (ode1) --------------------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', num2str(dt), 'StartTime', '0', 'StopTime', '10');
% ^ настройки решателя модели: фиксированный шаг, метод ode4 (RK4, не Эйлер), шаг = dt, старт с 0с, стоп 10с (потом переопределяется в run_v0.m)

if nargin < 3 || isempty(mdl_path)   % путь сохранения не передан?
    mdl_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models', 'gps_free_nav_v0.slx');
    % ^ путь по умолчанию: папка models/ рядом с папкой sim/
end
[mdl_dir, ~, ~] = fileparts(mdl_path);   % достаём папку из полного пути
if ~isfolder(mdl_dir)                     % такой папки ещё нет?
    mkdir(mdl_dir);                        % создаём её
end

% save_system() overwrites an existing file by renaming it to a ".bak"
% file first, then writing the new one - on Windows that rename can fail
% with "Access denied" if something else still has the old .slx open
% (a leftover MATLAB/Simulink handle from a previous run, antivirus, or
% a cloud-sync folder like OneDrive/Google Drive re-scanning it - the
% latter is common if the project lives under a synced folder such as
% Documents/Videos/Desktop). Deleting the old file ourselves first avoids
% that rename step, and retrying with a short pause rides out the more
% common transient case (the previous run's file handle hasn't been
% released yet).
% Русское резюме: save_system() при перезаписи сначала переименовывает
% старый файл в ".bak", а это переименование на Windows иногда падает с
% "Access denied", если файл всё ещё чем-то занят (антивирус, облачная
% синхронизация папки - OneDrive и т.п., или не до конца закрывшийся
% предыдущий запуск). Удаляем старый файл сами (тогда переименовывать
% нечего) и пробуем сохранить несколько раз с небольшой паузой.
if isfile(mdl_path)                 % модель уже сохранялась сюда раньше?
    try
        delete(mdl_path);            % удаляем старый файл - тогда save_system нечего переименовывать
    catch
        % не получилось удалить - ничего страшного, ниже всё равно попробуем сохранить с повторами
    end
end

max_attempts = 5;    % сколько раз пробовать сохранить, прежде чем сдаться
saved = false;        % пока не сохранили
last_err = [];        % последняя ошибка (для сообщения, если так и не получится)
for attempt = 1:max_attempts             % пробуем сохранить несколько раз
    try
        save_system(mdl, mdl_path);       % сохраняем собранную модель в файл .slx
        saved = true;                      % получилось - выходим из цикла
        break
    catch ME
        last_err = ME;                     % запоминаем ошибку
        pause(0.5);                        % небольшая пауза - вдруг файл как раз сейчас освобождается
    end
end
if ~saved   % так и не удалось сохранить после всех попыток?
    error('build_model:saveFailed', ...
        ['Could not save %s after %d attempts (%s).\nThis is almost always ' ...
         'Windows file locking, not a bug in the model itself - likely causes: ' ...
         'the model is still open in another Simulink window, antivirus is ' ...
         'scanning the file, or the project folder is inside a cloud-synced ' ...
         'folder (OneDrive/Google Drive/Dropbox - Documents/Videos/Desktop are ' ...
         'often auto-synced on Windows). Close any other Simulink window for this ' ...
         'model and try again; if it keeps happening, move the project to a ' ...
         'plain local folder (e.g. C:\\dev\\simulink) outside of any synced folder.'], ...
        mdl_path, max_attempts, last_err.message);
end
end

% =========================================================================
function build_sensor_subsystem(subPath, dt, params)
%BUILD_SENSOR_SUBSYSTEM Add the IMU error-model subsystem at SUBPATH.
% Inputs:  1 = a_true, 2 = w_true
% Outputs: 1 = a_meas, 2 = w_meas
% a_meas = a_true + (bias0 + bias random walk) + white noise, and
% likewise for w_meas - see sim/sensor_params.m for the parameters and
% README.md for the model description.
% Русское резюме: строим подсистему "модель датчика" - на вход
% истинные ускорение/угл.скорость, на выходе - зашумлённые (измеренные).

add_block('built-in/Subsystem', subPath);   % создаём пустую подсистему

add_block('simulink/Sources/In1', [subPath '/a_true']);   % вход подсистемы №1 - истинное ускорение
set_param([subPath '/a_true'], 'Port', '1', 'Position', pos(0, 0, 30, 14));   % это порт номер 1
add_block('simulink/Sources/In1', [subPath '/w_true']);   % вход подсистемы №2 - истинная угловая скорость
set_param([subPath '/w_true'], 'Port', '2', 'Position', pos(0, 3, 30, 14));   % это порт номер 2

build_channel(subPath, 'accel', dt, params.accel_bias0_std, ...
    params.accel_bias_rw_std, params.accel_noise_std, 1, [23341 23342]);
% ^ строим канал ошибок для акселерометра (параметры из sensor_params.m)
build_channel(subPath, 'gyro', dt, params.gyro_bias0_std, ...
    params.gyro_bias_rw_std, params.gyro_noise_std, 4, [23343 23344]);
% ^ строим канал ошибок для гироскопа (параметры из sensor_params.m)

add_line(subPath, 'a_true/1', 'accel_sum/1', 'autorouting', 'on');   % провод: истинное ускорение -> сумматор канала акселерометра
add_line(subPath, 'w_true/1', 'gyro_sum/1', 'autorouting', 'on');    % провод: истинная угл.скорость -> сумматор канала гироскопа

add_block('simulink/Sinks/Out1', [subPath '/a_meas']);   % выход подсистемы №1 - измеренное ускорение
set_param([subPath '/a_meas'], 'Port', '1', 'Position', pos(4, 0, 30, 14));   % это порт номер 1
add_line(subPath, 'accel_sum/1', 'a_meas/1', 'autorouting', 'on');   % провод: результат сумматора акселерометра -> выход

add_block('simulink/Sinks/Out1', [subPath '/w_meas']);   % выход подсистемы №2 - измеренная угловая скорость
set_param([subPath '/w_meas'], 'Port', '2', 'Position', pos(4, 3, 30, 14));   % это порт номер 2
add_line(subPath, 'gyro_sum/1', 'w_meas/1', 'autorouting', 'on');   % провод: результат сумматора гироскопа -> выход
end

function build_channel(subPath, name, dt, bias0, bias_rw_std, noise_std, rowOffset, seeds)
% One IMU channel: constant bias0 + (white-noise-driven) bias random
% walk + additive white measurement noise, summed with the true signal
% by the caller (accel_sum / gyro_sum), which this function creates.
% Русское резюме: строим один "канал" ошибок датчика (для акселерометра
% или для гироскопа) - постоянное смещение + плывущее смещение + шум,
% всё складывается с истинным сигналом.

bn = [subPath '/' name '_bias_noise'];   % путь к блоку белого шума, "разгоняющего" плывущее смещение
add_block('simulink/Sources/Band-Limited White Noise', bn);   % добавляем блок белого шума
set_param(bn, 'Position', pos(1, rowOffset, 70, 30));   % положение блока на схеме
try_set_param(bn, 'Cov', sprintf('%.10g', bias_rw_std^2), ...
    'Ts', num2str(dt), 'seed', num2str(seeds(1)));
% ^ мощность шума (из параметра "скорость плывания" смещения), шаг времени, случайное зерно

% Discrete (not continuous) integrator: the whole sensor/mechanization
% pipeline is meant to run at the single fixed rate dt, and a continuous
% state here would make everything downstream inherit continuous sample
% time - which the INS Mechanization block (it holds state in a
% persistent variable) is not allowed to do.
bi = [subPath '/' name '_bias_rw'];   % путь к блоку-интегратору (превращает шум в "плывущее" смещение)
add_block('simulink/Discrete/Discrete-Time Integrator', bi);   % добавляем ДИСКРЕТНЫЙ интегратор (важно - не непрерывный)
set_param(bi, 'Position', pos(2, rowOffset, 60, 30));   % положение блока на схеме
try_set_param(bi, 'SampleTime', num2str(dt));   % шаг времени интегратора = dt
add_line(subPath, [name '_bias_noise/1'], [name '_bias_rw/1'], 'autorouting', 'on');   % провод: шум -> вход интегратора

b0 = [subPath '/' name '_bias0'];   % путь к блоку постоянного смещения "при включении"
add_block('simulink/Sources/Constant', b0);   % добавляем константный блок
set_param(b0, 'Value', sprintf('%.10g', bias0), 'Position', pos(1, rowOffset + 1, 60, 30));   % значение = bias0

mn = [subPath '/' name '_meas_noise'];   % путь к блоку белого шума измерения (добавляется на каждом отсчёте)
add_block('simulink/Sources/Band-Limited White Noise', mn);   % добавляем блок белого шума
set_param(mn, 'Position', pos(1, rowOffset + 2, 70, 30));   % положение блока на схеме
try_set_param(mn, 'Cov', sprintf('%.10g', noise_std^2 * dt), ...
    'Ts', num2str(dt), 'seed', num2str(seeds(2)));
% ^ мощность шума (из параметра "шум измерения"), шаг времени, случайное зерно (другое, чем у bias_noise)

sm = [subPath '/' name '_sum'];   % путь к блоку-сумматору канала
add_block('simulink/Math Operations/Sum', sm);   % добавляем блок суммирования
set_param(sm, 'Inputs', '++++', 'Position', pos(3, rowOffset, 30, 60));   % 4 входа, все со знаком "плюс"

add_line(subPath, [name '_bias_rw/1'], [name '_sum/2'], 'autorouting', 'on');       % провод: плывущее смещение -> вход 2 суммы
add_line(subPath, [name '_bias0/1'], [name '_sum/3'], 'autorouting', 'on');         % провод: постоянное смещение -> вход 3 суммы
add_line(subPath, [name '_meas_noise/1'], [name '_sum/4'], 'autorouting', 'on');    % провод: шум измерения -> вход 4 суммы
% (вход 1 суммы - истинный сигнал, подключается снаружи в build_sensor_subsystem)
end

function set_chart_script(blockPath, script)
% Insert SCRIPT as the body of the MATLAB Function block at BLOCKPATH.
% See the "Create MATLAB Function Block Programmatically" pattern in the
% MATLAB documentation (MATLAB Function blocks are implemented as
% Stateflow charts under the hood).
% Русское резюме: вставляем текст кода script внутрь блока MATLAB
% Function по указанному пути (эти блоки на самом деле реализованы
% через Stateflow, поэтому используем именно такой способ).
try
    rt = sfroot;                                                   % корень объектной модели Stateflow
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath); % находим нужный блок по пути
    chart.Script = script;                                          % вставляем в него текст кода
catch ME
    warning('build_model:chartScript', ...
        ['Could not set the code for %s automatically (%s).\n' ...
         'Open the model, double-click that block, and paste the code from ' ...
         'sim/blocks/ manually instead.'], blockPath, ME.message);
    % ^ если не получилось - не прерываем сборку всей модели, а только предупреждаем
end
end

function try_set_param(blk, varargin)
% set_param wrapper that warns (instead of erroring out and aborting the
% whole build) if a parameter name/value is not accepted - e.g. if the
% Band-Limited White Noise block's parameter names differ slightly in
% your MATLAB version. Open the block dialog and set Noise power /
% Sample time / Seed by hand if you see this warning.
% Русское резюме: обёртка над set_param, которая при ошибке не рушит
% всю сборку, а только предупреждает (на случай, если имена параметров
% чуть отличаются в другой версии MATLAB).
try
    set_param(blk, varargin{:});   % пробуем задать параметры блока как обычно
catch ME
    warning('build_model:setParam', ...
        ['Could not set parameters on %s (%s).\n' ...
         'Set them manually in the block dialog: Noise power / Sample time / Seed.'], ...
        blk, ME.message);
    % ^ не получилось - предупреждаем, но не прерываем сборку
end
end

function p = pos(col, row, w, h)
% Simple grid layout helper: column/row -> [left top right bottom].
% Русское резюме: вспомогательная функция - переводит "номер колонки и
% строки" в координаты блока на схеме [слева сверху справа снизу].
left = 30 + col * 180;      % левая граница блока
top  = 30 + row * 90;       % верхняя граница блока
p = [left, top, left + w, top + h];   % итоговый прямоугольник положения блока
end
