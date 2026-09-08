function [xm, ym, edge_idx, mismatch] = map_match_nearest(x, y, nodes, edges)
%MAP_MATCH_NEAREST Snap a point to the nearest road segment on the map.
%   [XM, YM, EDGE_IDX, MISMATCH] = MAP_MATCH_NEAREST(X, Y, NODES, EDGES)
%   projects the point (X,Y) onto every road segment described by
%   NODES/EDGES (see map/generate_map.m for the format) and returns the
%   closest projected point (XM,YM), the index of the matched segment,
%   and the projection distance MISMATCH.
%
%   This is the "v0" (zero version) map-matching strategy: a pure
%   nearest-point projection with no memory of previous matches and no
%   constraint to stay on a connected path. It is enough to demonstrate
%   the concept (pulling a drifting dead-reckoning estimate back onto
%   the road network) but will occasionally jump to the wrong road near
%   intersections. A more robust version (v1+) would add continuity,
%   e.g. a Hidden Markov Model over the road graph - see README.md.
%
%   This function is a simplified reference copy for reading/exploring
%   the idea. The block actually used in the model
%   (sim/blocks/map_matching_block.m) adds one refinement on top of
%   this: it sticks to the currently matched road unless another one is
%   closer by a margin, so the match doesn't flip-flop between two
%   close roads near an intersection - without that, the plotted path
%   visibly cuts a diagonal shortcut through the intersection instead
%   of following one road to the corner.
%
% Русское резюме: справочная (не используемая напрямую в модели) копия
% простейшей привязки к карте - проецируем точку на все дороги и берём
% ближайшую. Реальный блок в модели (map_matching_block.m) делает то
% же самое, но ещё и с "гистерезисом" (не дёргается между дорогами).

best_dist = inf;                % пока лучшего варианта нет
xm = x; ym = y; edge_idx = 0;   % по умолчанию - сама точка (на случай, если дорог вообще нет)

for e = 1:size(edges, 1)                    % перебираем все дороги
    a = nodes(edges(e, 1), :);               % координаты первого узла этой дороги
    b = nodes(edges(e, 2), :);               % координаты второго узла этой дороги
    ab = b - a;                              % вектор дороги (из a в b)
    denom = dot(ab, ab);                     % квадрат длины дороги
    if denom < eps                            % дорога нулевой длины?
        t = 0;                                 % тогда проекция - это точка a
    else
        t = dot([x, y] - a, ab) / denom;       % параметр проекции на прямую (0=начало, 1=конец)
        t = min(max(t, 0), 1);                 % обрезаем в пределах отрезка дороги
    end
    proj = a + t * ab;                       % точка проекции на дороге
    d = hypot(x - proj(1), y - proj(2));     % расстояние от исходной точки до проекции
    if d < best_dist                          % это расстояние - новый рекорд (ближе всех пока)?
        best_dist = d;                          % запоминаем рекорд
        xm = proj(1); ym = proj(2);             % и координаты этой проекции
        edge_idx = e;                            % и номер этой дороги
    end
end

mismatch = best_dist;   % итоговое расстояние до ближайшей дороги
end
