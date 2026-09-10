function map = generate_map_3x3(save_path)
%GENERATE_MAP_3X3 Build a simple 3x3 synthetic road network.
%   MAP = GENERATE_MAP_3X3() builds a 3x3 grid of intersections (100 m
%   spacing), keeps every grid road (no dead ends removed - a 3x3 grid
%   is small enough already), and adds one diagonal avenue corner-to-
%   corner, same idea as generate_map.m's bigger 5x5 map.
%
%   MAP = GENERATE_MAP_3X3(SAVE_PATH) saves to a custom path instead.
%
%   The map is a graph:
%       map.nodes  - N x 2 array of [x y] intersection coordinates (m)
%       map.edges  - M x 2 array of node-index pairs; each row is one
%                    straight road segment between two intersections
%
%   This map exists specifically so the same road layout can be built
%   by hand in an external game: the exact node coordinates and edge
%   list are written out in map/MAP_3X3_INFO.md. Recordings exported
%   from a game that used that same layout (see truth/load_trajectory_
%   from_csv.m, sim/run_from_csv.m) will then land on real roads here
%   instead of drifting off a map the game never saw.
%
% Русское резюме: строит простую карту 3x3 (9 перекрёстков, шаг 100 м),
% без удаления дорог (сетка и так маленькая), плюс одна диагональная
% дорога из угла в угол. Нужна для того, чтобы точно такую же карту
% можно было вручную повторить в игре - координаты узлов и список дорог
% расписаны в map/MAP_3X3_INFO.md.

if nargin < 1 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'road_map_3x3.mat'); % путь сохранения по умолчанию
end

spacing = 100; % meters between adjacent intersections
% ^ м - расстояние между соседними перекрёстками
n = 3;         % n x n grid of intersections
% ^ размер сетки перекрёстков (3 на 3)

% Node index for grid position (i,j), i,j in 0..n-1, is
% idx(i,j) = i*n + j + 1 (i = x index, j = y index) - same convention as
% generate_map.m, so this map's numbering is predictable.
% Русское резюме: номер узла по координатам сетки (i,j) - та же формула,
% что и в generate_map.m.
nodes = zeros(n*n, 2);   % массив координат всех перекрёстков (пока нули)
k = 0;                     % счётчик узлов
for i = 0:n-1               % перебираем координату сетки по x
    for j = 0:n-1            % перебираем координату сетки по y
        k = k + 1;            % следующий номер узла
        nodes(k, :) = [i, j] * spacing; % координаты узла в метрах = индекс * расстояние между перекрёстками
    end
end
idx = @(i, j) i*n + j + 1;   % функция-помощник: координаты сетки (i,j) -> номер узла

edges = zeros(0, 2);          % список дорог (пар номеров узлов), пока пустой
for i = 0:n-1                  % перебираем координату сетки по x
    for j = 0:n-1                % перебираем координату сетки по y
        if i < n-1                 % есть сосед справа (по x)?
            edges(end+1, :) = [idx(i, j), idx(i+1, j)]; %#ok<AGROW>  % добавляем дорогу до соседа справа
        end
        if j < n-1                 % есть сосед сверху (по y)?
            edges(end+1, :) = [idx(i, j), idx(i, j+1)]; %#ok<AGROW>  % добавляем дорогу до соседа сверху
        end
    end
end

% One diagonal avenue cutting across the grid corner-to-corner, same as
% in generate_map.m - gives map matching something non-axis-aligned to
% handle even on this small a map.
% Русское резюме: добавляем одну диагональную дорогу из угла в угол -
% как и в generate_map.m.
edges(end+1, :) = [idx(0, 0), idx(n-1, n-1)]; %#ok<AGROW>   % дорога от нижнего левого до верхнего правого угла

map.nodes = nodes;      % записываем координаты узлов в результат
map.edges = edges;      % записываем список дорог в результат
map.spacing = spacing;  % записываем расстояние между перекрёстками (для справки)

save(save_path, 'map');   % сохраняем карту в .mat файл
end
