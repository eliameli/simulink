function map = generate_map(save_path)
%GENERATE_MAP Build a moderately complex synthetic road network for v0.
%   MAP = GENERATE_MAP() builds a 5x5 grid of city blocks (100 m
%   spacing between adjacent intersections), removes a handful of
%   segments to break the perfect-lattice regularity (dead ends and a
%   gap, closer to a real city layout), and adds one diagonal avenue
%   cutting straight across the grid. Saves it to map/road_map.mat next
%   to this file.
%
%   MAP = GENERATE_MAP(SAVE_PATH) saves to a custom path instead.
%
%   The map is a graph:
%       map.nodes  - N x 2 array of [x y] intersection coordinates (m)
%       map.edges  - M x 2 array of node-index pairs; each row is one
%                    straight road segment between two intersections
%
%   Still synthetic (not real map data) so the v0 pipeline can be built
%   and tested without any external map source or extra toolboxes, but
%   bigger and less regular than a plain grid, so a drawn route has
%   real choices to make (dead ends, a diagonal shortcut) and map
%   matching has real forks to get right. See README.md for how to
%   swap in real map data later.
%
% Русское резюме: строим синтетическую карту дорог - сетку 5x5
% перекрёстков, убираем несколько отрезков (чтобы были тупики, а не
% идеальная решётка) и добавляем одну диагональную дорогу через всю
% карту. Карта - это граф: map.nodes (координаты перекрёстков) и
% map.edges (какие перекрёстки соединены дорогой).

if nargin < 1 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'road_map.mat'); % путь сохранения по умолчанию
end

spacing = 100; % meters between adjacent intersections
% ^ м - расстояние между соседними перекрёстками
n = 5;         % n x n grid of intersections
% ^ размер сетки перекрёстков (5 на 5)

% Node index for grid position (i,j), i,j in 0..n-1, is
% idx(i,j) = i*n + j + 1 (i = x index, j = y index).
% Русское резюме: номер узла по координатам сетки (i,j) считается по формуле ниже.
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

% Remove a few segments to break the perfect-grid regularity: dead
% ends and a gap, instead of every intersection connecting in all four
% directions.
% Русское резюме: убираем несколько дорог, чтобы получились тупики -
% карта не идеальная решётка, а больше похожа на реальный город.
remove = [ ...
    idx(1,1) idx(1,2); ...
    idx(2,2) idx(2,3); ...
    idx(3,1) idx(4,1); ...
    idx(0,3) idx(1,3); ...
    idx(2,0) idx(3,0); ...
];   % список дорог "на удаление" (пары узлов)
for r = 1:size(remove, 1)      % перебираем каждую дорогу из списка на удаление
    hit = (edges(:,1) == remove(r,1) & edges(:,2) == remove(r,2)) | ...
          (edges(:,1) == remove(r,2) & edges(:,2) == remove(r,1));  % находим эту дорогу в списке всех дорог (в любом порядке узлов)
    edges(hit, :) = [];          % удаляем найденную дорогу из списка
end

% One diagonal avenue cutting across the grid corner-to-corner (not
% axis-aligned) - the map/matching code already handles any straight
% segment, this just gives it something other than horizontal/vertical
% roads to deal with, and a genuine shortcut a drawn route can use.
% Русское резюме: добавляем одну диагональную дорогу из угла в угол -
% код привязки к карте одинаково работает с любыми прямыми дорогами,
% просто теперь есть не только горизонтальные/вертикальные.
edges(end+1, :) = [idx(0, 0), idx(n-1, n-1)]; %#ok<AGROW>   % дорога от нижнего левого до верхнего правого угла

map.nodes = nodes;      % записываем координаты узлов в результат
map.edges = edges;      % записываем список дорог в результат
map.spacing = spacing;  % записываем расстояние между перекрёстками (для справки)

save(save_path, 'map');   % сохраняем карту в .mat файл
end
