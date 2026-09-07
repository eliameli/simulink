function [xm, ym, edge_idx, mismatch] = map_matching(x, y, nodes, edges)
%#codegen
% Map Matching (nearest road segment) - MATLAB Function block.
%
% This is the exact code build_model.m tries to insert into the
% "Map Matching" block of gps_free_nav_v0.slx. If the automatic
% insertion fails, open the model, double-click that (empty) MATLAB
% Function block, and paste this whole file's contents in.
%
% v0 map-matching strategy: project the dead-reckoned estimate (x,y)
% onto every road segment in the map (nodes/edges graph) and snap to
% whichever segment is closest. No memory of previous matches, no
% connectivity constraint - simple on purpose, see README.md for how to
% make this more robust later.

n_edges = size(edges, 1);
best_dist = inf;
xm = x; ym = y; edge_idx = 0;

for e = 1:n_edges
    a = nodes(edges(e,1), :);
    b = nodes(edges(e,2), :);
    ab = b - a;
    denom = ab(1)^2 + ab(2)^2;
    if denom < eps
        t = 0;
    else
        t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom;
        t = min(max(t, 0), 1);
    end
    proj = a + t*ab;
    d = hypot(x - proj(1), y - proj(2));
    if d < best_dist
        best_dist = d;
        xm = proj(1); ym = proj(2);
        edge_idx = e;
    end
end

mismatch = best_dist;
end
