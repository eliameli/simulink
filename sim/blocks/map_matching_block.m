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

persistent cur_edge
if isempty(cur_edge)
    cur_edge = 0; % no match yet
end

margin = 3; % m - how much closer another road must be before switching
n_edges = size(edges, 1);

best_dist = inf; best_x = x; best_y = y; best_edge = 0;
for e = 1:n_edges
    [px, py, d] = project_to_segment(x, y, nodes(edges(e,1),:), nodes(edges(e,2),:));
    if d < best_dist
        best_dist = d; best_x = px; best_y = py; best_edge = e;
    end
end

if cur_edge ~= 0
    [cx, cy, cd] = project_to_segment(x, y, nodes(edges(cur_edge,1),:), nodes(edges(cur_edge,2),:));
    if cd <= best_dist + margin
        best_edge = cur_edge; best_x = cx; best_y = cy; best_dist = cd;
    end
end

cur_edge = best_edge;
xm = best_x; ym = best_y; edge_idx = best_edge; mismatch = best_dist;
end

function [px, py, d] = project_to_segment(x, y, a, b)
ab = b - a;
denom = ab(1)^2 + ab(2)^2;
if denom < eps
    t = 0;
else
    t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom;
    t = min(max(t, 0), 1);
end
px = a(1) + t*ab(1);
py = a(2) + t*ab(2);
d = hypot(x - px, y - py);
end
