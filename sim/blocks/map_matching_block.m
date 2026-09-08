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
% whichever segment is closest - BUT stick to the currently matched
% road unless a different one is closer by more than MARGIN. Without
% this, near an intersection two roads can be almost equally close, so
% the match flips between them from one sample to the next, and the
% plotted path visibly cuts a diagonal shortcut through the
% intersection instead of following one road to the corner and turning.
% This still has no real path-continuity constraint (a full version
% would be a Hidden Markov Model over the road graph, see README.md),
% just enough memory to stop flip-flopping between two close roads.
%
% Русское резюме: на каждом шаге проецируем оценённую позицию (x,y) на
% все дороги карты и берём ближайшую - но не переключаемся с уже
% выбранной дороги, пока другая не окажется ближе с заметным запасом
% (MARGIN). Без этого запаса у перекрёстка, где две дороги почти
% одинаково близко, выбор "дёргался" бы каждый шаг, и на графике линия
% срезала бы перекрёсток по диагонали вместо поворота по дороге.

persistent cur_edge      % номер дороги, выбранной на прошлом шаге (для гистерезиса)
if isempty(cur_edge)      % это первый вызов блока за весь прогон?
    cur_edge = 0;          % ещё ни одна дорога не выбрана
end

margin = 3; % m - how much closer another road must be before switching
% ^ м - насколько другая дорога должна быть ближе, чтобы мы на неё переключились
n_edges = size(edges, 1); % сколько всего дорог на карте

best_dist = inf; best_x = x; best_y = y; best_edge = 0; % пока лучшего варианта нет
for e = 1:n_edges                                        % перебираем все дороги
    [px, py, d] = project_to_segment(x, y, nodes(edges(e,1),:), nodes(edges(e,2),:)); % проекция точки на эту дорогу
    if d < best_dist                                       % это расстояние - новый рекорд (ближе всех пока)?
        best_dist = d; best_x = px; best_y = py; best_edge = e; % запоминаем как лучший вариант
    end
end

if cur_edge ~= 0                        % на прошлом шаге уже была выбрана какая-то дорога?
    [cx, cy, cd] = project_to_segment(x, y, nodes(edges(cur_edge,1),:), nodes(edges(cur_edge,2),:)); % проекция на ту же дорогу сейчас
    if cd <= best_dist + margin          % она всё ещё не сильно хуже самой близкой (в пределах запаса)?
        best_edge = cur_edge; best_x = cx; best_y = cy; best_dist = cd; % тогда остаёмся на ней (не переключаемся)
    end
end

cur_edge = best_edge;    % запоминаем выбор дороги для следующего шага (гистерезис)
xm = best_x; ym = best_y; edge_idx = best_edge; mismatch = best_dist; % отдаём наружу: точку на дороге, номер дороги и расстояние до неё
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
