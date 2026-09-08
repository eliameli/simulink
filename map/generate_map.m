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

if nargin < 1 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'road_map.mat');
end

spacing = 100; % meters between adjacent intersections
n = 5;         % n x n grid of intersections

% Node index for grid position (i,j), i,j in 0..n-1, is
% idx(i,j) = i*n + j + 1 (i = x index, j = y index).
nodes = zeros(n*n, 2);
k = 0;
for i = 0:n-1
    for j = 0:n-1
        k = k + 1;
        nodes(k, :) = [i, j] * spacing;
    end
end
idx = @(i, j) i*n + j + 1;

edges = zeros(0, 2);
for i = 0:n-1
    for j = 0:n-1
        if i < n-1
            edges(end+1, :) = [idx(i, j), idx(i+1, j)]; %#ok<AGROW>
        end
        if j < n-1
            edges(end+1, :) = [idx(i, j), idx(i, j+1)]; %#ok<AGROW>
        end
    end
end

% Remove a few segments to break the perfect-grid regularity: dead
% ends and a gap, instead of every intersection connecting in all four
% directions.
remove = [ ...
    idx(1,1) idx(1,2); ...
    idx(2,2) idx(2,3); ...
    idx(3,1) idx(4,1); ...
    idx(0,3) idx(1,3); ...
    idx(2,0) idx(3,0); ...
];
for r = 1:size(remove, 1)
    hit = (edges(:,1) == remove(r,1) & edges(:,2) == remove(r,2)) | ...
          (edges(:,1) == remove(r,2) & edges(:,2) == remove(r,1));
    edges(hit, :) = [];
end

% One diagonal avenue cutting across the grid corner-to-corner (not
% axis-aligned) - the map/matching code already handles any straight
% segment, this just gives it something other than horizontal/vertical
% roads to deal with, and a genuine shortcut a drawn route can use.
edges(end+1, :) = [idx(0, 0), idx(n-1, n-1)]; %#ok<AGROW>

map.nodes = nodes;
map.edges = edges;
map.spacing = spacing;

save(save_path, 'map');
end
