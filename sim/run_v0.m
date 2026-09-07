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

%% 2) Ground-truth trajectory: drive through these intersections in order
% (see map/generate_map.m for how node indices map to (x,y); with the
% default 3x3 grid this path makes an "L" shape with two turns)
path_nodes = [1 4 5 6 9];
v_cruise   = 8;     % m/s cruise speed (~29 km/h)
dt         = 0.01;  % s, fixed step - used for both the truth generation
                     % and the Simulink model, they must match
truth = generate_trajectory(map, path_nodes, v_cruise, dt);

%% 3) Build the Simulink model (only once - reused on later runs)
mdl = 'gps_free_nav_v0';
mdl_path = fullfile(here, '..', 'models', 'gps_free_nav_v0.slx');
if isfile(mdl_path)
    load_system(mdl_path);
else
    params = sensor_params();
    build_model(dt, params, mdl_path);
end

%% 4) Provide the model's inputs (it reads these base-workspace variables)
a_true_ts = [truth.t, truth.a_true]; %#ok<NASGU> (used by the From Workspace block)
w_true_ts = [truth.t, truth.w_true]; %#ok<NASGU>
map_nodes = map.nodes;               %#ok<NASGU>
map_edges = map.edges;               %#ok<NASGU>

set_param(mdl, 'StopTime', num2str(truth.t(end)));

%% 5) Simulate
sim(mdl);
% Produces (in the base workspace, via the model's "To Workspace" blocks):
%   meas_log     - [a_meas, w_meas]        noisy sensor output
%   est_log      - [x,y,psi,v] estimate    raw dead reckoning (drifts!)
%   matched_log  - [xm, ym]                after snapping to the road map
%   mismatch_log - distance to the matched road segment
%   edge_log     - index of the matched road segment

%% 6) Compare true vs. dead-reckoned vs. map-matched trajectories
est     = est_log.Data;      % columns: x_est, y_est, psi_est, v_est
matched = matched_log.Data;  % columns: xm, ym

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
