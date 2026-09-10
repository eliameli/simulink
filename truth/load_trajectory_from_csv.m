function truth = load_trajectory_from_csv(csv_path)
%LOAD_TRAJECTORY_FROM_CSV Load a ground-truth trajectory recorded elsewhere
%(e.g. a custom game) instead of drawing one by hand with map/pick_route.m.
%   TRUTH = LOAD_TRAJECTORY_FROM_CSV(CSV_PATH) reads a CSV file with a
%   header row and columns t,x,y,psi,v,a,w (see
%   data/game_run_20260910T081331784Z.csv for a real example) and returns
%   a TRUTH struct in exactly the same shape that
%   truth/generate_trajectory.m produces, so it's a drop-in replacement
%   anywhere a TRUTH struct is expected:
%     t                 - time vector [s], must be uniformly spaced
%     a_true, w_true    - true forward accel [m/s^2] and yaw rate [rad/s]
%     x, y, psi, v       - true pose (m, m, rad, m/s)
%     waypoints          - start/end point, for reference/plotting
%     dt                 - the time step found in the file [s]; use this
%                          (not a hardcoded 0.01 like run_v0.m) when
%                          building the Simulink model, so the model's
%                          fixed step matches the recording's sample rate
%
%   Column meaning/units must match what map/generate_map.m and
%   sim/blocks/*.m expect: x,y in meters in the map's coordinate frame,
%   psi in radians (0 = +x axis, counter-clockwise positive), v the
%   forward speed, a = dv/dt, w = dpsi/dt.
%
% Русское резюме: читает CSV с реальной траекторией (например,
% выгруженной из игры) и оборачивает его в ту же структуру truth, что и
% generate_trajectory.m - можно использовать взаимозаменяемо с
% нарисованным мышью маршрутом.

T = readtable(csv_path);   % читаем CSV, колонки находятся по именам заголовка (t,x,y,psi,v,a,w)

% Simulink's fixed-step solver always starts its own clock at t=0. Many
% loggers don't write a sample at the very first instant (the first row
% here is t=0.02, not t=0), so without this shift the model would run
% for one extra step beyond the end of this data - a length mismatch
% between the logged signals and truth further down the pipeline.
% Русское резюме: симуляция в Simulink всегда стартует с t=0, а запись
% в игре может начинаться не с нуля (первая строка - t=0.02) - сдвигаем
% время так, чтобы первая точка была t=0, иначе модель сделает на один
% шаг больше, чем есть строк в файле, и массивы не совпадут по длине.
t = T.t - T.t(1);

truth.t      = t;        % время (сдвинуто так, что начинается с 0)
truth.x      = T.x;      % координата x
truth.y      = T.y;      % координата y
truth.psi    = T.psi;    % курс
truth.v      = T.v;      % скорость
truth.a_true = T.a;      % истинное ускорение
truth.w_true = T.w;      % истинная угловая скорость

% Same sanity check the rest of the pipeline relies on implicitly (both
% generate_trajectory.m and the Simulink model assume one fixed dt) -
% catch a non-uniform or out-of-order recording here with a clear error
% instead of silently feeding a bad step size into the model downstream.
% Русское резюме: проверяем, что шаг времени в файле действительно
% постоянный (остальной код это предполагает) - иначе явная ошибка
% сразу, а не непонятный сбой модели позже.
dt_all = diff(truth.t);
dt = median(dt_all);
if any(dt_all <= 0) || max(abs(dt_all - dt)) > 1e-3
    error('load_trajectory_from_csv:nonUniformTime', ...
        ['Time column in %s is not a uniform, increasing sequence ' ...
         '(expected a constant step of about %.4f s). Resample the ' ...
         'recording to a fixed rate before loading it.'], csv_path, dt);
end
truth.dt = dt;   % шаг времени - берём из самих данных

truth.waypoints = [truth.x(1), truth.y(1); truth.x(end), truth.y(end)];
% ^ начальная и конечная точки - только для справки/отрисовки, как и у generate_trajectory.m
end
