function waypoints = pick_route(map)
%PICK_ROUTE Click a driving route on the map with the mouse.
%   WAYPOINTS = PICK_ROUTE(MAP) shows the road network and lets you
%   left-click a sequence of points to define the route the vehicle
%   drives (roughly along the roads, though nothing forces that -
%   this is v0, click wherever you want the truth trajectory to go).
%   Press Enter (or right-click / Esc) when you have at least 2 points
%   to finish. Returns an N x 2 array of [x y] waypoints, in click order.

figure('Name', 'Click your route');
hold on; axis equal; grid on;
plot_map(map);
xlabel('x, m'); ylabel('y, m');
title({'Left-click waypoints along the route you want to drive', ...
       'Press Enter (or right-click) when done - need at least 2 points'});

[x, y] = ginput; %#ok<ASGLU> click, click, ... Enter to finish

while numel(x) < 2
    warndlg('Need at least 2 points - click again.', 'Not enough points');
    [x, y] = ginput;
end

waypoints = [x, y];
plot(waypoints(:,1), waypoints(:,2), 'm.-', 'MarkerSize', 20, 'LineWidth', 1.5, ...
    'DisplayName', 'Your route');
legend('Location', 'best');
end
