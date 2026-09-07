function map = generate_map(save_path)
%GENERATE_MAP Build a simple synthetic road network for v0.
%   MAP = GENERATE_MAP() builds a 3x3 grid of straight roads (like a
%   small city block grid), 100 m between adjacent intersections, and
%   saves it to map/road_map.mat next to this file.
%
%   MAP = GENERATE_MAP(SAVE_PATH) saves to a custom path instead.
%
%   The map is a graph:
%       map.nodes  - N x 2 array of [x y] intersection coordinates (m)
%       map.edges  - M x 2 array of node-index pairs; each row is one
%                    straight road segment between two intersections
%
%   This is intentionally synthetic (not real map data) so the v0
%   pipeline can be built and tested without any external map source or
%   extra toolboxes. See README.md for how to swap in real map data later.

if nargin < 1 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'road_map.mat');
end

spacing = 100; % meters between adjacent intersections

% Build a 3x3 grid of nodes. Node index for grid position (i,j),
% i,j in 0..2, is idx(i,j) = i*3 + j + 1 (i = x index, j = y index).
nodes = zeros(9, 2);
k = 0;
for i = 0:2
    for j = 0:2
        k = k + 1;
        nodes(k, :) = [i, j] * spacing;
    end
end
idx = @(i, j) i*3 + j + 1;

edges = zeros(0, 2);
for i = 0:2
    for j = 0:2
        if i < 2
            edges(end+1, :) = [idx(i, j), idx(i+1, j)]; %#ok<AGROW>
        end
        if j < 2
            edges(end+1, :) = [idx(i, j), idx(i, j+1)]; %#ok<AGROW>
        end
    end
end

map.nodes = nodes;
map.edges = edges;
map.spacing = spacing;

save(save_path, 'map');
end
