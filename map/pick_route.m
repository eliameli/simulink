function waypoints = pick_route(map)
%PICK_ROUTE Draw a driving route on the map by dragging the mouse.
%   WAYPOINTS = PICK_ROUTE(MAP) shows the road network. Press and HOLD
%   the left mouse button, drag along the route you want to drive, then
%   release the button when done. Returns an N x 2 array of [x y]
%   waypoints, spaced ~WAYPOINT_SPACING meters apart along the drawn
%   stroke - a fixed distance, not a fixed number of mouse events.
%
%   Why: WindowButtonMotionFcn fires roughly once per real mouse
%   movement, so a slow, careful drag racks up far more raw samples
%   than a fast one over the same drawn distance - and, worse, the
%   small hand-tremor that comes with moving slowly adds real
%   back-and-forth wobble that inflates the drawn path's total length,
%   so a slow drag could end up describing a route noticeably longer
%   than what it looks like on screen. Fixed with two passes: first
%   DECIMATE_DIST strips out that sub-few-meter jitter (points closer
%   together than this are dropped, so wobble in place stops adding
%   length), then the survivors are resampled to land every
%   WAYPOINT_SPACING meters along the cleaned-up path - so the final
%   waypoint count depends only on how far you actually drew, not on
%   drawing speed or raw sample count.

fig = figure('Name', 'Draw your route');
hold on; axis equal; grid on;
plot_map(map);
xlabel('x, m'); ylabel('y, m');
title({'Press and HOLD the left mouse button, drag along your route,', ...
       'release when done'});

raw = zeros(0, 2);
drawing = false;
h_line = plot(NaN, NaN, 'm.-', 'MarkerSize', 10, 'LineWidth', 1.5, ...
    'DisplayName', 'Your route');

set(fig, 'WindowButtonDownFcn', @on_down);
set(fig, 'WindowButtonMotionFcn', @on_move);
set(fig, 'WindowButtonUpFcn', @on_up);

uiwait(fig);

if size(raw, 1) < 2
    error('pick_route:tooFewPoints', ...
        'Route too short - hold the mouse button and drag further before releasing.');
end

decimate_dist   = 3;  % m - strips hand-tremor jitter before resampling
waypoint_spacing = 25; % m - final spacing between waypoints, ~4 per 100 m

decimated = decimate_points(raw, decimate_dist);
waypoints = resample_by_arclength(decimated, waypoint_spacing);

set(h_line, 'XData', waypoints(:,1), 'YData', waypoints(:,2), 'Marker', 'o');
legend('Location', 'best');

    function on_down(~, ~)
        drawing = true;
        cp = get(gca, 'CurrentPoint');
        raw = cp(1, 1:2);
        set(h_line, 'XData', raw(:,1), 'YData', raw(:,2));
    end

    function on_move(~, ~)
        if drawing
            cp = get(gca, 'CurrentPoint');
            raw(end+1, :) = cp(1, 1:2); %#ok<AGROW>
            set(h_line, 'XData', raw(:,1), 'YData', raw(:,2));
        end
    end

    function on_up(~, ~)
        drawing = false;
        set(fig, 'WindowButtonDownFcn', '', 'WindowButtonMotionFcn', '', 'WindowButtonUpFcn', '');
        uiresume(fig);
    end
end

function kept = decimate_points(raw, min_dist)
% Keep a point only if it's at least MIN_DIST from the last kept one -
% strips near-duplicate/jittery samples without touching real path
% shape (real segments are much longer than MIN_DIST).
kept = raw(1, :);
for i = 2:size(raw, 1)
    if norm(raw(i, :) - kept(end, :)) >= min_dist
        kept(end+1, :) = raw(i, :); %#ok<AGROW>
    end
end
if norm(raw(end, :) - kept(end, :)) > 0
    kept(end+1, :) = raw(end, :);
end
end

function wp = resample_by_arclength(points, spacing)
% Walk the polyline PONTS by cumulative distance and place one waypoint
% every SPACING meters (linearly interpolated between the surrounding
% points), plus the exact endpoint - so waypoint count and spacing
% depend only on total path length, not on how many points came in.
n = size(points, 1);
if n < 2
    wp = points;
    return
end

seg_len = vecnorm(diff(points), 2, 2);
cum_len = [0; cumsum(seg_len)];
total_len = cum_len(end);

if total_len < spacing
    wp = points([1, end], :);
    return
end

targets = (0:spacing:total_len)';
if targets(end) < total_len
    targets(end+1) = total_len;
end

wp = zeros(numel(targets), 2);
seg_idx = 1;
for i = 1:numel(targets)
    d = targets(i);
    while seg_idx < n - 1 && cum_len(seg_idx + 1) < d
        seg_idx = seg_idx + 1;
    end
    L0 = cum_len(seg_idx);
    L1 = cum_len(seg_idx + 1);
    if L1 > L0
        t = (d - L0) / (L1 - L0);
    else
        t = 0;
    end
    wp(i, :) = points(seg_idx, :) + t * (points(seg_idx + 1, :) - points(seg_idx, :));
end
end
