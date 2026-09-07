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
%   This function is a reference copy: the equivalent logic is embedded
%   directly inside the "Map Matching" MATLAB Function block of the
%   Simulink model (see sim/blocks/map_matching_block.m) so it can run
%   inside the model without calling out to this file.

best_dist = inf;
xm = x; ym = y; edge_idx = 0;

for e = 1:size(edges, 1)
    a = nodes(edges(e, 1), :);
    b = nodes(edges(e, 2), :);
    ab = b - a;
    denom = dot(ab, ab);
    if denom < eps
        t = 0;
    else
        t = dot([x, y] - a, ab) / denom;
        t = min(max(t, 0), 1);
    end
    proj = a + t * ab;
    d = hypot(x - proj(1), y - proj(2));
    if d < best_dist
        best_dist = d;
        xm = proj(1); ym = proj(2);
        edge_idx = e;
    end
end

mismatch = best_dist;
end
