function waypoints = pick_route(map)
%PICK_ROUTE Draw a driving route on the map by dragging the mouse.
%   WAYPOINTS = PICK_ROUTE(MAP) shows the road network. Press and HOLD
%   the left mouse button, drag along the route you want to drive, then
%   release the button when done. Returns an N x 2 array of [x y]
%   waypoints, spaced ~WAYPOINT_SPACING meters apart along the drawn
%   stroke - a fixed distance, not a fixed number of mouse events.
%
%   Why: WindowButtonMotionFcn fires roughly once per real mouse
%   movement, so a slow, careful drag racks up far more raw samples
%   than a fast one over the same drawn distance - and, worse, the
%   small hand-tremor that comes with moving slowly adds real
%   back-and-forth wobble that inflates the drawn path's total length,
%   so a slow drag could end up describing a route noticeably longer
%   than what it looks like on screen. Fixed with two passes: first
%   DECIMATE_DIST strips out that sub-few-meter jitter (points closer
%   together than this are dropped, so wobble in place stops adding
%   length), then the survivors are resampled to land every
%   WAYPOINT_SPACING meters along the cleaned-up path - so the final
%   waypoint count depends only on how far you actually drew, not on
%   drawing speed or raw sample count.
%
% Русское резюме: рисуем маршрут мышью (зажать -> вести -> отпустить).
% "Сырые" точки от движения мыши сначала прореживаем (убираем дрожь
% руки), потом расставляем заново строго через равные метры вдоль
% пути - так итоговое число точек не зависит от того, быстро или
% медленно вы вели мышь.

fig = figure('Name', 'Draw your route');        % открываем окно для рисования маршрута
hold on; axis equal; grid on;                     % не стирать предыдущие графики; равный масштаб осей; сетка
plot_map(map);                                    % рисуем дорожную сеть
xlabel('x, m'); ylabel('y, m');                   % подписи осей
title({'Press and HOLD the left mouse button, drag along your route,', ...
       'release when done'});                     % инструкция для пользователя

raw = zeros(0, 2);        % сюда будем копить "сырые" точки от движения мыши (пока пусто)
drawing = false;           % флаг "кнопка мыши сейчас зажата" (пока нет)
h_line = plot(NaN, NaN, 'm.-', 'MarkerSize', 10, 'LineWidth', 1.5, ...
    'DisplayName', 'Your route');   % линия маршрута на графике (пока пустая, будет обновляться вживую)

set(fig, 'WindowButtonDownFcn', @on_down);   % при нажатии кнопки мыши - вызвать on_down
set(fig, 'WindowButtonMotionFcn', @on_move); % при движении мыши - вызвать on_move
set(fig, 'WindowButtonUpFcn', @on_up);       % при отпускании кнопки - вызвать on_up

uiwait(fig);   % ждём, пока пользователь не отпустит кнопку (on_up разбудит нас через uiresume)

if size(raw, 1) < 2                                    % накопилось меньше двух точек?
    error('pick_route:tooFewPoints', ...
        'Route too short - hold the mouse button and drag further before releasing.'); % маршрут слишком короткий - ошибка
end

decimate_dist   = 3;  % m - strips hand-tremor jitter before resampling
% ^ м - порог прореживания (убирает дрожь руки перед основной расстановкой точек)
waypoint_spacing = 25; % m - final spacing between waypoints, ~4 per 100 m
% ^ м - итоговое расстояние между точками маршрута (~4 точки на 100 м)

decimated = decimate_points(raw, decimate_dist);          % шаг 1: убираем мелкую дрожь руки
waypoints = resample_by_arclength(decimated, waypoint_spacing); % шаг 2: расставляем точки заново, строго через равные метры

set(h_line, 'XData', waypoints(:,1), 'YData', waypoints(:,2), 'Marker', 'o'); % показываем итоговые точки маршрута (кружками)
legend('Location', 'best');   % включаем легенду

    function on_down(~, ~)
        drawing = true;                    % кнопка мыши зажата - начинаем рисовать
        cp = get(gca, 'CurrentPoint');      % текущие координаты курсора на графике
        raw = cp(1, 1:2);                   % это первая точка маршрута
        set(h_line, 'XData', raw(:,1), 'YData', raw(:,2)); % обновляем линию на графике
    end

    function on_move(~, ~)
        if drawing                          % кнопка всё ещё зажата?
            cp = get(gca, 'CurrentPoint');   % текущие координаты курсора
            raw(end+1, :) = cp(1, 1:2); %#ok<AGROW>  % добавляем новую точку в конец списка
            set(h_line, 'XData', raw(:,1), 'YData', raw(:,2)); % обновляем линию на графике (видно вживую)
        end
    end

    function on_up(~, ~)
        drawing = false;    % кнопку отпустили - рисование закончено
        set(fig, 'WindowButtonDownFcn', '', 'WindowButtonMotionFcn', '', 'WindowButtonUpFcn', ''); % отключаем обработчики мыши
        uiresume(fig);       % будим uiwait выше - можно продолжать выполнение функции
    end
end

function kept = decimate_points(raw, min_dist)
% Keep a point only if it's at least MIN_DIST from the last kept one -
% strips near-duplicate/jittery samples without touching real path
% shape (real segments are much longer than MIN_DIST).
% Русское резюме: оставляем точку, только если она не ближе min_dist
% к последней оставленной - убирает дрожь руки, не портя сам маршрут.
kept = raw(1, :);                          % первая точка всегда остаётся
for i = 2:size(raw, 1)                     % проходим по всем остальным "сырым" точкам
    if norm(raw(i, :) - kept(end, :)) >= min_dist  % эта точка достаточно далеко от последней оставленной?
        kept(end+1, :) = raw(i, :); %#ok<AGROW>      % да - оставляем её тоже
    end
end
if norm(raw(end, :) - kept(end, :)) > 0    % последняя "сырая" точка ещё не была добавлена?
    kept(end+1, :) = raw(end, :);           % добавляем её принудительно (чтобы маршрут не обрывался раньше времени)
end
end

function wp = resample_by_arclength(points, spacing)
% Walk the polyline PONTS by cumulative distance and place one waypoint
% every SPACING meters (linearly interpolated between the surrounding
% points), plus the exact endpoint - so waypoint count and spacing
% depend only on total path length, not on how many points came in.
% Русское резюме: идём вдоль ломаной линии points по накопленному
% расстоянию и ставим точку каждые spacing метров (с интерполяцией
% между соседними исходными точками) - число итоговых точек зависит
% только от длины пути, а не от того, сколько исходных точек было.
n = size(points, 1);        % сколько точек на входе
if n < 2                     % меньше двух точек - пересчитывать нечего
    wp = points;              % возвращаем как есть
    return
end

seg_len = vecnorm(diff(points), 2, 2);  % длина каждого отрезка между соседними точками
cum_len = [0; cumsum(seg_len)];          % накопленное расстояние от начала пути до каждой точки
total_len = cum_len(end);                % общая длина всего пути

if total_len < spacing                   % путь короче одного шага расстановки?
    wp = points([1, end], :);             % тогда просто оставляем начало и конец
    return
end

targets = (0:spacing:total_len)';        % список "целевых" расстояний от начала: 0, spacing, 2*spacing, ...
if targets(end) < total_len              % последняя цель не долетает точно до конца пути?
    targets(end+1) = total_len;           % добавляем сам конец пути как последнюю точку
end

wp = zeros(numel(targets), 2);    % сюда сложим итоговые точки маршрута
seg_idx = 1;                       % с какого отрезка исходного пути начинаем поиск
for i = 1:numel(targets)           % идём по всем целевым расстояниям
    d = targets(i);                 % текущая целевая длина пути
    while seg_idx < n - 1 && cum_len(seg_idx + 1) < d  % сдвигаем текущий отрезок вперёд, пока целевая длина не попадёт в него
        seg_idx = seg_idx + 1;
    end
    L0 = cum_len(seg_idx);          % накопленная длина в начале текущего отрезка
    L1 = cum_len(seg_idx + 1);      % накопленная длина в конце текущего отрезка
    if L1 > L0                      % отрезок не нулевой длины?
        t = (d - L0) / (L1 - L0);    % доля пути внутри этого отрезка (0..1)
    else
        t = 0;                       % вырожденный случай - берём начало отрезка
    end
    wp(i, :) = points(seg_idx, :) + t * (points(seg_idx + 1, :) - points(seg_idx, :)); % интерполируем координаты
end
end
