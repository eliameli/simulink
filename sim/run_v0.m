%% run_v0.m
% GPS-free navigation, v0: map -> ground truth -> noisy IMU -> RK4 dead
% reckoning -> nearest-road map matching.
%
% Run this file directly in MATLAB R2024a (Run button, or F5, or type
% "run_v0" with sim/ as the current folder). See README.md for the big
% picture and what to try next.

close all; clear; clc;

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'map'));
addpath(fullfile(here, '..', 'truth'));

%% 1) Build the (synthetic, v0) road map
map = generate_map();

%% 2) Ground-truth trajectory: click your route on the map
% Left-click a sequence of waypoints, then press Enter (or right-click)
% to finish - see map/pick_route.m. The vehicle drives through them at
% a single constant speed (no acceleration/braking), turning at each
% intermediate waypoint.
waypoints = pick_route(map);
v_cruise  = 8;     % m/s, constant cruise speed (~29 km/h)
dt        = 0.01;  % s, fixed step - used for both the truth generation
                    % and the Simulink model, they must match
truth = generate_trajectory(waypoints, v_cruise, dt);

%% 3) (Re)build the Simulink model every run
% Always rebuilt from build_model.m rather than reused from disk: while
% this v0 pipeline is still being debugged, a stale .slx left over from
% an earlier version of build_model.m is a common source of confusing
% "it still fails the same way" reports even after the source is fixed.
% Rebuilding is fast, so there is no real cost to doing it every time.
mdl = 'gps_free_nav_v0';
mdl_path = fullfile(here, '..', 'models', 'gps_free_nav_v0.slx');

% If a model with this name is already loaded (e.g. from a different
% copy/folder of this project, or a previous run in this MATLAB
% session), Simulink refuses to load/overwrite it - close it first.
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

params = sensor_params();
build_model(dt, params, mdl_path);

%% 4) Provide the model's inputs (it reads these base-workspace variables)
a_true_ts = [truth.t, truth.a_true]; %#ok<NASGU> (used by the From Workspace block)
w_true_ts = [truth.t, truth.w_true]; %#ok<NASGU>
map_nodes = map.nodes;               %#ok<NASGU>
map_edges = map.edges;               %#ok<NASGU>

% Dead reckoning has to start from a known pose - use the truth
% trajectory's own starting point (this is what makes the INS
% Mechanization block work with whatever route you just clicked,
% instead of a route fixed at build time).
ins_x0   = truth.x(1);   %#ok<NASGU>
ins_y0   = truth.y(1);   %#ok<NASGU>
ins_psi0 = truth.psi(1); %#ok<NASGU>
ins_v0   = truth.v(1);   %#ok<NASGU>

set_param(mdl, 'StopTime', num2str(truth.t(end)));

%% 5) Simulate
% Capture the output explicitly instead of relying on the model's "To
% Workspace" blocks to land their variables in the base workspace on
% their own - whether they do depends on a model/MATLAB-version setting
% (Data Import/Export > "Return workspace outputs"), so this is the
% version-robust way to get logged signals back regardless of that.
simOut = sim(mdl);
% Produces, packaged in simOut (one per "To Workspace" block):
%   meas_log     - [a_meas, w_meas]        noisy sensor output
%   est_log      - [x,y,psi,v] estimate    raw dead reckoning (drifts!)
%   matched_log  - [xm, ym]                after snapping to the road map
%   mismatch_log - distance to the matched road segment
%   edge_log     - index of the matched road segment

%% 6) Compare true vs. dead-reckoned vs. map-matched trajectories
est     = simOut.get('est_log').Data;      % columns: x_est, y_est, psi_est, v_est
matched = simOut.get('matched_log').Data;  % columns: xm, ym

figure('Name', 'GPS-free navigation v0 - trajectory');
hold on; axis equal; grid on;
plot_map(map);
plot(truth.x, truth.y, 'k-', 'LineWidth', 2, 'DisplayName', 'Truth (exact)');
plot(est(:,1), est(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning (drifts)');
plot(matched(:,1), matched(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched');
legend('Location', 'best');
xlabel('x, m'); ylabel('y, m');
title('True vs. dead-reckoned vs. map-matched trajectory');

figure('Name', 'GPS-free navigation v0 - error');
err_dr = hypot(est(:,1) - truth.x, est(:,2) - truth.y);
err_mm = hypot(matched(:,1) - truth.x, matched(:,2) - truth.y);
plot(truth.t, err_dr, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning error'); hold on; grid on;
plot(truth.t, err_mm, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched error');
legend('Location', 'best');
xlabel('t, s'); ylabel('position error, m');
title('Position error vs. time');

fprintf('Final dead-reckoning error:  %.2f m\n', err_dr(end));
fprintf('Final map-matched error:     %.2f m\n', err_mm(end));
