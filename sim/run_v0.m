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

% How often the "green" trajectory (see build_model.m) snaps itself
% back onto the nearest road, instead of drifting open-loop like the
% raw ("red") dead reckoning does. It also snaps immediately (without
% waiting for the timer) whenever it has drifted more than
% gap_threshold_m away from the raw ("red") estimate.
periodic_correction_s = 1.5; % s %#ok<NASGU>
gap_threshold_m       = 15;  % m %#ok<NASGU>

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
%   green_log    - [x,y,psi,v] estimate    dead reckoning, periodically
%                                           snapped back onto the road
%   matched_log  - [xm, ym]                per-sample road snap applied
%                                           on top of green_log
%   mismatch_log - distance to the matched road segment
%   edge_log     - index of the matched road segment

%% 6) Compare true vs. dead-reckoned vs. periodically-corrected vs. map-matched
est     = simOut.get('est_log').Data;      % columns: x_est, y_est, psi_est, v_est
green   = simOut.get('green_log').Data;    % columns: x_g, y_g, psi_g, v_g
matched = simOut.get('matched_log').Data;  % columns: xm, ym
meas    = simOut.get('meas_log').Data;     % columns: a_meas, w_meas

figure('Name', 'GPS-free navigation v0 - trajectory');
hold on; axis equal; grid on;
plot_map(map);
plot(truth.x, truth.y, 'k-', 'LineWidth', 2, 'DisplayName', 'Truth (exact)');
plot(est(:,1), est(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning (drifts)');
plot(green(:,1), green(:,2), 'g-', 'LineWidth', 1.2, 'DisplayName', sprintf('Periodic snap every %.1fs', periodic_correction_s));
plot(matched(:,1), matched(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched (final)');
legend('Location', 'best');
xlabel('x, m'); ylabel('y, m');
title('True vs. dead-reckoned vs. periodically-corrected vs. map-matched trajectory');

figure('Name', 'GPS-free navigation v0 - error');
err_dr = hypot(est(:,1) - truth.x, est(:,2) - truth.y);
err_gr = hypot(green(:,1) - truth.x, green(:,2) - truth.y);
err_mm = hypot(matched(:,1) - truth.x, matched(:,2) - truth.y);
hold on; grid on;
plot(truth.t, err_dr, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Dead reckoning error');
plot(truth.t, err_gr, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Periodic-snap error');
plot(truth.t, err_mm, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Map-matched (final) error');
legend('Location', 'best');
xlabel('t, s'); ylabel('position error, m');
title('Position error vs. time');

fprintf('Final dead reckoning error:    %.2f m\n', err_dr(end));
fprintf('Final periodic-snap error:     %.2f m\n', err_gr(end));
fprintf('Final map-matched error:       %.2f m\n', err_mm(end));

%% 7) Diagnostics: does the gyro actually "see" each turn, and when does
% the estimated heading start to diverge from the true heading? Use
% these two plots to tell apart "the sensor/model missed a turn" (a
% real bug) from "dead reckoning had already drifted off course before
% this turn even happened" (expected - it's the whole reason map
% matching exists). unwrap() avoids fake +-360 degree jumps in the plot.
figure('Name', 'GPS-free navigation v0 - diagnostics');

subplot(2,1,1);
plot(truth.t, rad2deg(truth.w_true), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True yaw rate');
hold on; grid on;
plot(truth.t, rad2deg(meas(:,2)), 'm-', 'DisplayName', 'Measured (noisy) yaw rate');
legend('Location', 'best');
xlabel('t, s'); ylabel('yaw rate, deg/s');
title('Does the gyro see each turn? (spikes should line up with truth)');

subplot(2,1,2);
plot(truth.t, rad2deg(unwrap(truth.psi)), 'k-', 'LineWidth', 2, 'DisplayName', 'True heading');
hold on; grid on;
plot(truth.t, rad2deg(unwrap(est(:,3))), 'r--', 'DisplayName', 'Dead-reckoned heading');
plot(truth.t, rad2deg(unwrap(green(:,3))), 'g-', 'DisplayName', 'Periodic-snap heading');
legend('Location', 'best');
xlabel('t, s'); ylabel('heading (psi), deg');
title('Heading over time: look for a growing gap BEFORE each turn');

%% 8) Diagnostics: is green actually moving, or stuck?
% x(t) and y(t) separately (not just the x-y trajectory plot) make it
% obvious whether green is tracking red between corrections (it should
% move almost identically to red until the next snap) or is frozen.
figure('Name', 'GPS-free navigation v0 - green vs red over time');
subplot(2,1,1);
plot(truth.t, truth.x, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Truth x'); hold on; grid on;
plot(truth.t, est(:,1), 'r--', 'DisplayName', 'Dead-reckoned x');
plot(truth.t, green(:,1), 'g-', 'DisplayName', 'Periodic-snap x');
legend('Location', 'best');
xlabel('t, s'); ylabel('x, m');
title('x(t): is green actually moving between corrections?');

subplot(2,1,2);
plot(truth.t, truth.y, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Truth y'); hold on; grid on;
plot(truth.t, est(:,2), 'r--', 'DisplayName', 'Dead-reckoned y');
plot(truth.t, green(:,2), 'g-', 'DisplayName', 'Periodic-snap y');
legend('Location', 'best');
xlabel('t, s'); ylabel('y, m');
title('y(t): is green actually moving between corrections?');
