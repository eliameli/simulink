function truth = generate_trajectory(waypoints, v_cruise, dt, save_path)
%GENERATE_TRAJECTORY Build a ground-truth pose/control history for v0.
%   TRUTH = GENERATE_TRAJECTORY(WAYPOINTS, V_CRUISE, DT, SAVE_PATH) drives
%   a vehicle through WAYPOINTS (an N x 2 list of [x y] points, e.g. from
%   map/pick_route.m) at a single constant speed V_CRUISE, with a short
%   constant-rate in-place turn at every intermediate waypoint where the
%   heading changes. No acceleration/braking - speed is constant for the
%   whole run, only heading changes (accelerometer reads ~0 except
%   during a turn's centripetal blip; gyro reads the turn rate).
%
%   Because the "true" control input (forward acceleration a_true(t),
%   always 0 here, and yaw rate w_true(t)) is known analytically by
%   construction, the exact ground-truth pose is obtained by integrating
%   it with zero sensor error using the same RK4 integrator used for
%   dead reckoning (see unicycle_rk4_step.m) - this is what "precise/
%   exact coordinates" means here: not measured, but known by
%   construction.
%
%   Returns TRUTH with fields:
%     t                 - time vector [s]
%     a_true, w_true    - true forward accel [m/s^2] (always 0) and yaw
%                          rate [rad/s]
%     x, y, psi, v       - true pose (m, m, rad, m/s)
%     waypoints          - the input waypoints, for reference/plotting
%
% Русское резюме: строим "истинную" (эталонную) траекторию по точкам
% маршрута waypoints. Скорость всегда постоянна v_cruise, курс меняется
% только на поворотах в промежуточных точках. Так как весь план
% движения (когда разгон/поворот/etc) известен заранее и точно, эта
% траектория получается интегрированием без единой ошибки - именно
% поэтому она "истинная" (в отличие от того, что "видят" датчики).

if nargin < 2 || isempty(v_cruise), v_cruise = 8;    end   % м/с (~29 км/ч) - скорость по умолчанию
if nargin < 3 || isempty(dt),       dt = 0.01;       end   % с - шаг времени по умолчанию
if nargin < 4 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'truth_trajectory.mat'); % путь для сохранения по умолчанию
end

% Duration of a full 90-degree turn. Since speed is constant (no
% braking for corners), the vehicle keeps moving forward *while* it
% turns, covering some extra arc length during every heading change -
% so below, the turn's duration (and therefore its extra distance)
% scales with how sharp it actually is: a real 90-degree corner takes
% the full turn_time_90, but a 2-degree wobble (typical hand-drawn
% jitter from map/pick_route.m, which can produce dozens of waypoints
% along one route) takes almost none. Without this scaling, every one
% of those small wobbles would cost the same fixed time/distance as a
% real corner, and a hand-drawn route with many waypoints would end up
% far longer than the line you actually drew.
turn_time_90 = 0.5; % s, for a 90-degree turn
% ^ с - за сколько времени проходит полный поворот на 90 градусов (мелкие повороты - пропорционально быстрее)

nseg        = size(waypoints, 1) - 1;        % количество отрезков маршрута (точек минус один)
seg_vec     = diff(waypoints);               % векторы каждого отрезка (из одной точки в следующую)
seg_len     = vecnorm(seg_vec, 2, 2);        % длины отрезков
seg_heading = atan2(seg_vec(:,2), seg_vec(:,1)); % курс (направление) движения по каждому отрезку

% ---- Build a piecewise timeline of (duration, a, w) blocks ----
% Строим кусочный план движения: список блоков (длительность, ускорение, угловая скорость)
dur = []; a_blk = []; w_blk = []; % пустые массивы - будем заполнять по мере обхода маршрута

for k = 1:nseg                                 % идём по всем отрезкам маршрута по очереди
    dur(end+1)   = seg_len(k) / v_cruise; %#ok<AGROW>  % время проезда прямого отрезка = длина / скорость
    a_blk(end+1) = 0;                     %#ok<AGROW>  % ускорение на прямом участке - всегда 0 (скорость постоянна)
    w_blk(end+1) = 0;                     %#ok<AGROW>  % угловая скорость на прямом участке - 0 (курс не меняется)

    if k < nseg                                 % это не последний отрезок (значит, дальше есть поворот)?
        dpsi = wrap_to_pi(seg_heading(k+1) - seg_heading(k)); % на сколько меняется курс на этом повороте
        % Scale duration by |dpsi| (a 90-degree turn takes turn_time_90,
        % a tiny wobble takes proportionally less) - this keeps the
        % turn rate itself roughly constant regardless of how sharp the
        % corner is, which is closer to how a real vehicle turns anyway.
        this_turn_time = max(turn_time_90 * abs(dpsi) / (pi/2), dt); % длительность поворота - пропорционально углу
        dur(end+1)   = this_turn_time;        %#ok<AGROW>  % добавляем блок "поворот" в план
        a_blk(end+1) = 0;                     %#ok<AGROW>  % ускорение во время поворота - тоже 0
        w_blk(end+1) = dpsi / this_turn_time; %#ok<AGROW>  % угловая скорость поворота = угол / время (примерно постоянная)
    end
end

% ---- Sample (a_true, w_true) on a uniform time grid ----
% Build directly from an integer sample count per block, NOT a
% time-window mask (t >= tb & t < tb+dur(k)): a mask can silently drop
% an entire block if its window happens to fall between two grid
% points (a real risk for a very short block - e.g. a small heading
% wobble whose duration is clamped near one sample by the scaling
% above), because floating-point tb rarely lands exactly on a multiple
% of dt. Rounding each block to whole samples guarantees every block,
% however short, contributes at least one real sample - no gyro turn
% event can vanish from the signal.
% Русское резюме: превращаем план "блоками" в массив отсчётов с шагом
% dt - через ЦЕЛОЕ число отсчётов на каждый блок (не через маску по
% времени), чтобы ни один, даже самый короткий, поворот не потерялся.
n_samples = max(1, round(dur / dt));  % сколько отсчётов приходится на каждый блок (минимум 1)
a_parts = cell(1, numel(dur));         % сюда сложим кусочки массива ускорения по каждому блоку
w_parts = cell(1, numel(dur));         % сюда сложим кусочки массива угловой скорости по каждому блоку
for k = 1:numel(dur)                   % проходим по всем блокам плана
    a_parts{k} = repmat(a_blk(k), n_samples(k), 1); % повторяем значение ускорения n_samples(k) раз
    w_parts{k} = repmat(w_blk(k), n_samples(k), 1); % повторяем значение угловой скорости n_samples(k) раз
end
a_true = cat(1, a_parts{:});   % склеиваем все кусочки ускорения в один массив
w_true = cat(1, w_parts{:});   % склеиваем все кусочки угловой скорости в один массив
t = (0:numel(a_true))' * dt; % one more point than control samples (initial state + one per step)
% ^ массив времени - на одну точку больше, чем массив управления (начальное состояние + один момент на шаг)
% Pad a_true/w_true by one repeated sample so they're the same length
% as t - convenient for plotting them against t directly. The RK4 loop
% below only ever reads indices 1..numel(t)-1, so this padded last
% sample is never actually used for integration.
a_true(end+1) = a_true(end); %#ok<AGROW>  % дублируем последнее значение ускорения - для одинаковой длины с t (для графиков)
w_true(end+1) = w_true(end); %#ok<AGROW>  % дублируем последнее значение угл. скорости - для одинаковой длины с t

% ---- Integrate exactly (RK4, zero sensor error) for the true pose ----
% Starts already moving at v_cruise (constant speed for the whole run).
s0 = [waypoints(1,1); waypoints(1,2); seg_heading(1); v_cruise]; % начальное состояние: первая точка маршрута, начальный курс, скорость v_cruise
S = zeros(4, numel(t));    % сюда сложим всю траекторию состояний (по столбцу на каждый момент времени)
S(:,1) = s0;                % первый столбец - начальное состояние
for kk = 1:numel(t)-1        % шагаем по всем моментам времени
    S(:,kk+1) = unicycle_rk4_step(S(:,kk), a_true(kk), w_true(kk), dt); % RK4-шаг: следующее состояние из текущего
end

truth.t         = t;          % массив времени
truth.a_true    = a_true;     % истинное ускорение во времени
truth.w_true    = w_true;     % истинная угловая скорость во времени
truth.x         = S(1,:)';    % истинная координата x во времени
truth.y         = S(2,:)';    % истинная координата y во времени
truth.psi       = S(3,:)';    % истинный курс во времени
truth.v         = S(4,:)';    % истинная скорость во времени
truth.waypoints = waypoints;  % исходные точки маршрута (для справки/отрисовки)

save(save_path, 'truth');  % сохраняем результат в .mat файл
end

function a = wrap_to_pi(a)
% Wrap an angle to (-pi, pi]. Implemented locally to avoid a dependency
% on wrapToPi, which lives in a toolbox that may not be licensed.
% Русское резюме: приводим угол к диапазону (-pi, pi] - свой вариант,
% чтобы не зависеть от wrapToPi (эта функция из тулбокса, которого
% может не быть в лицензии).
a = mod(a + pi, 2*pi) - pi;
end
