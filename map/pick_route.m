function waypoints = pick_route(map)
%PICK_ROUTE Draw a driving route on the map by dragging the mouse.
%   WAYPOINTS = PICK_ROUTE(MAP) shows the road network. Press and HOLD
%   the left mouse button, drag along the route you want to drive, then
%   release the button when done. Returns an N x 2 array of [x y]
%   waypoints, resampled along the drawn stroke (at least MIN_DIST
%   apart, so a slow/jittery drag doesn't produce thousands of
%   near-zero-length segments).

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

min_dist = 5; % m - minimum spacing between kept waypoints
waypoints = raw(1, :);
for i = 2:size(raw, 1)
    if norm(raw(i, :) - waypoints(end, :)) >= min_dist
        waypoints(end+1, :) = raw(i, :); %#ok<AGROW>
    end
end
if norm(raw(end, :) - waypoints(end, :)) > 0
    waypoints(end+1, :) = raw(end, :);
end

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
