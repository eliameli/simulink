function [xm, ym, edge_idx, mismatch] = map_matching(x, y, nodes, edges)
%#codegen
% Map Matching (nearest road segment, with hysteresis) - MATLAB Function
% block.
%
% This is the exact code build_model.m tries to insert into the
% "Map Matching" block of gps_free_nav_v0.slx. If the automatic
% insertion fails, open the model, double-click that (empty) MATLAB
% Function block, and paste this whole file's contents in.
%
% v0 map-matching strategy: project the dead-reckoned estimate (x,y)
% onto every road segment in the map (nodes/edges graph) and snap to
% whichever segment is closest - BUT (1) stick to the currently matched
% road unless a different one is closer by more than MARGIN, and (2)
% only ever switch to a road that actually touches the current one (or,
% if the estimate has drifted far enough that even the roads touching
% the current one are all farther than LOST_THRESHOLD, fall back to
% searching the whole map again, so a bad state can't get stuck forever).
%
% (1) alone (no adjacency restriction) is what an earlier version of
% this did. Verified against a real dead-reckoning run (see
% sim/blocks/ins_mechanization_block.m for how that's generated) in a
% standalone Python re-implementation before writing this: whenever the
% estimate happened to be briefly closer to some unrelated, disconnected
% road than to anything actually reachable from the current one - e.g.
% the diagonal avenue and a horizontal street that don't meet anywhere
% near that point - the old logic would snap straight to it, and the
% plotted path visibly cut a shortcut through open space between two
% roads that don't even intersect there. Restricting candidates to
% roads that share a node with the current one (falling back to a full
% search only when truly "lost") keeps the plotted path always
% following an actual connected sequence of roads instead.
%
% Русское резюме: на каждом шаге проецируем оценённую позицию (x,y) на
% все дороги карты, но НЕ выбираем просто самую близкую дорогу - только
% среди текущей дороги и тех, что физически соединены с ней через общий
% узел (плюс гистерезис - запас MARGIN, чтобы не дёргаться между двумя
% соседними). Если даже соединённые дороги все дальше LOST_THRESHOLD
% (оценка сильно "скакнула"), ищем заново по всей карте - иначе синяя
% линия могла бы навсегда застрять на устаревшей дороге. Без этого
% ограничения (проверено на реальной записи через Python) линия иногда
% перескакивала на близкую, но НЕ связанную с текущей дорогу - и на
% графике это выглядело как диагональный срез через газон между двумя
% дорогами, которые в этом месте вообще не пересекаются.

persistent cur_edge      % номер дороги, выбранной на прошлом шаге (для гистерезиса и проверки связности)
if isempty(cur_edge)      % это первый вызов блока за весь прогон?
    cur_edge = 0;          % ещё ни одна дорога не выбрана
end

margin = 3;          % м - насколько другая дорога должна быть ближе, чтобы мы на неё переключились
lost_threshold = 40; % м - если даже дороги, соединённые с текущей, дальше этого - считаем, что "потеряли" дорогу
n_edges = size(edges, 1); % сколько всего дорог на карте

% -- ближайшая дорога по ВСЕЙ карте (нужна всегда: на первом вызове и как запасной вариант, если "потерялись") --
best_dist = inf; best_x = x; best_y = y; best_edge = 0; % пока лучшего варианта нет
for e = 1:n_edges                                        % перебираем все дороги
    [px, py, d] = project_to_segment(x, y, nodes(edges(e,1),:), nodes(edges(e,2),:)); % проекция точки на эту дорогу
    if d < best_dist                                       % это расстояние - новый рекорд (ближе всех пока)?
        best_dist = d; best_x = px; best_y = py; best_edge = e; % запоминаем как лучший вариант
    end
end

if cur_edge == 0                         % это самый первый вызов (ещё нет "текущей" дороги)?
    chosen_edge = best_edge; chosen_x = best_x; chosen_y = best_y; chosen_dist = best_dist; % берём глобально ближайшую
else
    % -- ищем лучшую дорогу только среди текущей и тех, что с ней соединены (через общий узел) --
    a1 = edges(cur_edge, 1); a2 = edges(cur_edge, 2); % два узла текущей дороги
    local_best_dist = inf; local_best_x = x; local_best_y = y; local_best_edge = 0; % пока лучшего локального варианта нет
    for e = 1:n_edges                                        % перебираем все дороги ещё раз
        is_adjacent = (e == cur_edge) || edges(e,1) == a1 || edges(e,1) == a2 || ...
            edges(e,2) == a1 || edges(e,2) == a2; % это текущая дорога, или у неё есть общий узел с текущей?
        if ~is_adjacent                                       % не связана с текущей дорогой?
            continue                                            % пропускаем - её нельзя выбрать напрямую
        end
        [px, py, d] = project_to_segment(x, y, nodes(edges(e,1),:), nodes(edges(e,2),:)); % проекция на эту (связанную) дорогу
        if d < local_best_dist                                % это расстояние - новый рекорд среди связанных дорог?
            local_best_dist = d; local_best_x = px; local_best_y = py; local_best_edge = e; % запоминаем
        end
    end

    if local_best_dist <= lost_threshold   % связанная "окрестность" текущей дороги всё ещё не слишком далеко?
        if local_best_edge ~= cur_edge       % лучшая связанная дорога - не та же, что была?
            [cx, cy, cd] = project_to_segment(x, y, nodes(edges(cur_edge,1),:), nodes(edges(cur_edge,2),:)); % проекция на прежнюю дорогу
            if cd <= local_best_dist + margin  % прежняя дорога всё ещё не сильно хуже (гистерезис)?
                local_best_edge = cur_edge; local_best_x = cx; local_best_y = cy; local_best_dist = cd; % остаёмся на прежней
            end
        end
        chosen_edge = local_best_edge; chosen_x = local_best_x; chosen_y = local_best_y; chosen_dist = local_best_dist;
    else
        % "потеряли" дорогу - оценка положения слишком далеко от всей связной окрестности прежней дороги
        chosen_edge = best_edge; chosen_x = best_x; chosen_y = best_y; chosen_dist = best_dist; % ищем заново по всей карте
    end
end

cur_edge = chosen_edge;    % запоминаем выбор дороги для следующего шага
xm = chosen_x; ym = chosen_y; edge_idx = chosen_edge; mismatch = chosen_dist; % отдаём наружу: точку на дороге, номер дороги и расстояние до неё
end

function [px, py, d] = project_to_segment(x, y, a, b)
% Проекция точки (x,y) на отрезок дороги от точки a до точки b.
ab = b - a;                  % вектор дороги (из a в b)
denom = ab(1)^2 + ab(2)^2;   % квадрат длины дороги
if denom < eps                % дорога нулевой длины (вырожденный случай)?
    t = 0;                     % тогда проекция - это просто точка a
else
    t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom; % параметр проекции на прямую (0=начало, 1=конец)
    t = min(max(t, 0), 1);      % обрезаем в пределах отрезка (не выходим за концы дороги)
end
px = a(1) + t*ab(1);   % x проекции
py = a(2) + t*ab(2);   % y проекции
d = hypot(x - px, y - py); % расстояние от исходной точки до её проекции на дорогу
end
