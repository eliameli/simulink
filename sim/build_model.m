function mdl_path = build_model(dt, params, mdl_path)
%BUILD_MODEL Programmatically build the v0 Simulink model.
%   MDL_PATH = BUILD_MODEL(DT, PARAMS, MDL_PATH) builds
%   gps_free_nav_v0.slx: a synthetic IMU sensor model feeding an RK4
%   dead-reckoning ("INS mechanization") block, followed by a
%   nearest-road-segment map-matching block, with signal logging to the
%   base workspace. Saves it to MDL_PATH and returns that path.
%
%   DT     - fixed simulation step [s] (must match the truth trajectory's
%            time step, see truth/generate_trajectory.m)
%   PARAMS - sensor error parameters, see sim/sensor_params.m. These
%            values are baked into the model at build time: if you
%            change sensor_params.m, rebuild the model (delete the .slx
%            or call build_model again) for the change to take effect.
%
%   At simulation time (not build time) the caller must set these
%   base-workspace variables before calling sim():
%     a_true_ts, w_true_ts - [time, value] matrices, the true forward
%                             accel / yaw rate the sensor model corrupts
%     map_nodes, map_edges - the road map (see map/generate_map.m)
%   See sim/run_v0.m for a complete example.
%
%   NOTE: this script was written and reviewed without a local MATLAB to
%   test it against (see README.md). The two MATLAB Function blocks are
%   inserted via the documented Stateflow scripting API; if that step
%   fails for your MATLAB version, open the model and paste the code
%   from sim/blocks/*.m into the corresponding (then-empty) block by
%   hand - the rest of the model (wiring, sources, sinks) is built with
%   plain, very stable block-diagram APIs and should not need that.

mdl = 'gps_free_nav_v0';

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
open_system(mdl);

% ---- top-level sources -------------------------------------------------
add_block('simulink/Sources/From Workspace', [mdl '/a_true']);
set_param([mdl '/a_true'], 'VariableName', 'a_true_ts', ...
    'SampleTime', num2str(dt), 'Position', pos(0, 0, 90, 30));

add_block('simulink/Sources/From Workspace', [mdl '/w_true']);
set_param([mdl '/w_true'], 'VariableName', 'w_true_ts', ...
    'SampleTime', num2str(dt), 'Position', pos(0, 1, 90, 30));

% ---- Sensor Model subsystem --------------------------------------------
sensPath = [mdl '/Sensor Model'];
build_sensor_subsystem(sensPath, dt, params);
set_param(sensPath, 'Position', pos(1, 0.5, 140, 90));

add_line(mdl, 'a_true/1', 'Sensor Model/1', 'autorouting', 'on');
add_line(mdl, 'w_true/1', 'Sensor Model/2', 'autorouting', 'on');

% ---- Ts constant (fixed step size, fed into the mechanization block) --
add_block('simulink/Sources/Constant', [mdl '/Ts']);
set_param([mdl '/Ts'], 'Value', sprintf('%.10g', dt), ...
    'Position', pos(2, 2.3, 60, 30));

% ---- INS Mechanization (RK4) MATLAB Function block ---------------------
mechPath = [mdl '/INS Mechanization (RK4)'];
add_block('simulink/User-Defined Functions/MATLAB Function', mechPath);
set_param(mechPath, 'Position', pos(2.3, 0.5, 160, 100));
% This block uses a persistent variable to hold its running pose
% estimate, which Simulink does not allow on a block that inherits a
% continuous sample time (its inputs run through the Sensor Model
% subsystem's continuous bias-random-walk Integrator, so without this
% the block would try to inherit continuous time). Force it to run at
% the model's fixed discrete step instead - that's what the RK4
% mechanization is designed for anyway. (Sample time is a property of
% the underlying Stateflow chart object, not a plain block parameter -
% set it together with the script.)
set_chart_script(mechPath, fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'blocks', 'ins_mechanization_block.m')), dt);

add_line(mdl, 'Sensor Model/1', 'INS Mechanization (RK4)/1', 'autorouting', 'on');
add_line(mdl, 'Sensor Model/2', 'INS Mechanization (RK4)/2', 'autorouting', 'on');
add_line(mdl, 'Ts/1', 'INS Mechanization (RK4)/3', 'autorouting', 'on');

% ---- Map data constants -------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/Map Nodes']);
set_param([mdl '/Map Nodes'], 'Value', 'map_nodes', 'Position', pos(3.6, 2, 90, 30));

add_block('simulink/Sources/Constant', [mdl '/Map Edges']);
set_param([mdl '/Map Edges'], 'Value', 'map_edges', 'Position', pos(3.6, 2.6, 90, 30));

% ---- Map Matching MATLAB Function block ---------------------------------
matchPath = [mdl '/Map Matching'];
add_block('simulink/User-Defined Functions/MATLAB Function', matchPath);
set_param(matchPath, 'Position', pos(4, 0.5, 160, 100));
% Explicit discrete sample time for consistency with the rest of the
% fixed-step pipeline (not strictly required - this block has no
% persistent state - but keeps every stage running at the same rate).
set_chart_script(matchPath, fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'blocks', 'map_matching_block.m')), dt);

add_line(mdl, 'INS Mechanization (RK4)/1', 'Map Matching/1', 'autorouting', 'on');
add_line(mdl, 'INS Mechanization (RK4)/2', 'Map Matching/2', 'autorouting', 'on');
add_line(mdl, 'Map Nodes/1', 'Map Matching/3', 'autorouting', 'on');
add_line(mdl, 'Map Edges/1', 'Map Matching/4', 'autorouting', 'on');

% ---- Logging: Mux each group of signals, then To Workspace -------------
add_block('simulink/Signal Routing/Mux', [mdl '/Mux Meas']);
set_param([mdl '/Mux Meas'], 'Inputs', '2', 'Position', pos(3.6, -0.6, 20, 60));
add_line(mdl, 'Sensor Model/1', 'Mux Meas/1', 'autorouting', 'on');
add_line(mdl, 'Sensor Model/2', 'Mux Meas/2', 'autorouting', 'on');
add_block('simulink/Sinks/To Workspace', [mdl '/meas_log']);
set_param([mdl '/meas_log'], 'VariableName', 'meas_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(4.6, -0.6, 90, 30));
add_line(mdl, 'Mux Meas/1', 'meas_log/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Est']);
set_param([mdl '/Mux Est'], 'Inputs', '4', 'Position', pos(3.2, 0.5, 20, 80));
add_line(mdl, 'INS Mechanization (RK4)/1', 'Mux Est/1', 'autorouting', 'on');
add_line(mdl, 'INS Mechanization (RK4)/2', 'Mux Est/2', 'autorouting', 'on');
add_line(mdl, 'INS Mechanization (RK4)/3', 'Mux Est/3', 'autorouting', 'on');
add_line(mdl, 'INS Mechanization (RK4)/4', 'Mux Est/4', 'autorouting', 'on');
add_block('simulink/Sinks/To Workspace', [mdl '/est_log']);
set_param([mdl '/est_log'], 'VariableName', 'est_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(3.4, 3.6, 90, 30));
add_line(mdl, 'Mux Est/1', 'est_log/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Matched']);
set_param([mdl '/Mux Matched'], 'Inputs', '2', 'Position', pos(5, 0.3, 20, 60));
add_line(mdl, 'Map Matching/1', 'Mux Matched/1', 'autorouting', 'on');
add_line(mdl, 'Map Matching/2', 'Mux Matched/2', 'autorouting', 'on');
add_block('simulink/Sinks/To Workspace', [mdl '/matched_log']);
set_param([mdl '/matched_log'], 'VariableName', 'matched_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.6, 0.3, 90, 30));
add_line(mdl, 'Mux Matched/1', 'matched_log/1', 'autorouting', 'on');

add_block('simulink/Sinks/To Workspace', [mdl '/mismatch_log']);
set_param([mdl '/mismatch_log'], 'VariableName', 'mismatch_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.6, 1.3, 90, 30));
add_line(mdl, 'Map Matching/4', 'mismatch_log/1', 'autorouting', 'on');

add_block('simulink/Sinks/To Workspace', [mdl '/edge_log']);
set_param([mdl '/edge_log'], 'VariableName', 'edge_log', ...
    'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', 'Position', pos(5.6, 2.3, 90, 30));
add_line(mdl, 'Map Matching/3', 'edge_log/1', 'autorouting', 'on');

% ---- Solver: fixed-step RK4 (ode4), NOT Euler (ode1) --------------------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', num2str(dt), 'StartTime', '0', 'StopTime', '10');

if nargin < 3 || isempty(mdl_path)
    mdl_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models', 'gps_free_nav_v0.slx');
end
[mdl_dir, ~, ~] = fileparts(mdl_path);
if ~isfolder(mdl_dir)
    mkdir(mdl_dir);
end
save_system(mdl, mdl_path);
end

% =========================================================================
function build_sensor_subsystem(subPath, dt, params)
%BUILD_SENSOR_SUBSYSTEM Add the IMU error-model subsystem at SUBPATH.
% Inputs:  1 = a_true, 2 = w_true
% Outputs: 1 = a_meas, 2 = w_meas
% a_meas = a_true + (bias0 + bias random walk) + white noise, and
% likewise for w_meas - see sim/sensor_params.m for the parameters and
% README.md for the model description.

add_block('built-in/Subsystem', subPath);

add_block('simulink/Sources/In1', [subPath '/a_true']);
set_param([subPath '/a_true'], 'Port', '1', 'Position', pos(0, 0, 30, 14));
add_block('simulink/Sources/In1', [subPath '/w_true']);
set_param([subPath '/w_true'], 'Port', '2', 'Position', pos(0, 3, 30, 14));

build_channel(subPath, 'accel', dt, params.accel_bias0_std, ...
    params.accel_bias_rw_std, params.accel_noise_std, 1, [23341 23342]);
build_channel(subPath, 'gyro', dt, params.gyro_bias0_std, ...
    params.gyro_bias_rw_std, params.gyro_noise_std, 4, [23343 23344]);

add_line(subPath, 'a_true/1', 'accel_sum/1', 'autorouting', 'on');
add_line(subPath, 'w_true/1', 'gyro_sum/1', 'autorouting', 'on');

add_block('simulink/Sinks/Out1', [subPath '/a_meas']);
set_param([subPath '/a_meas'], 'Port', '1', 'Position', pos(4, 0, 30, 14));
add_line(subPath, 'accel_sum/1', 'a_meas/1', 'autorouting', 'on');

add_block('simulink/Sinks/Out1', [subPath '/w_meas']);
set_param([subPath '/w_meas'], 'Port', '2', 'Position', pos(4, 3, 30, 14));
add_line(subPath, 'gyro_sum/1', 'w_meas/1', 'autorouting', 'on');
end

function build_channel(subPath, name, dt, bias0, bias_rw_std, noise_std, rowOffset, seeds)
% One IMU channel: constant bias0 + (white-noise-driven) bias random
% walk + additive white measurement noise, summed with the true signal
% by the caller (accel_sum / gyro_sum), which this function creates.

bn = [subPath '/' name '_bias_noise'];
add_block('simulink/Sources/Band-Limited White Noise', bn);
set_param(bn, 'Position', pos(1, rowOffset, 70, 30));
try_set_param(bn, 'Cov', sprintf('%.10g', bias_rw_std^2), ...
    'Ts', num2str(dt), 'seed', num2str(seeds(1)));

bi = [subPath '/' name '_bias_rw'];
add_block('simulink/Continuous/Integrator', bi);
set_param(bi, 'Position', pos(2, rowOffset, 60, 30));
add_line(subPath, [name '_bias_noise/1'], [name '_bias_rw/1'], 'autorouting', 'on');

b0 = [subPath '/' name '_bias0'];
add_block('simulink/Sources/Constant', b0);
set_param(b0, 'Value', sprintf('%.10g', bias0), 'Position', pos(1, rowOffset + 1, 60, 30));

mn = [subPath '/' name '_meas_noise'];
add_block('simulink/Sources/Band-Limited White Noise', mn);
set_param(mn, 'Position', pos(1, rowOffset + 2, 70, 30));
try_set_param(mn, 'Cov', sprintf('%.10g', noise_std^2 * dt), ...
    'Ts', num2str(dt), 'seed', num2str(seeds(2)));

sm = [subPath '/' name '_sum'];
add_block('simulink/Math Operations/Sum', sm);
set_param(sm, 'Inputs', '++++', 'Position', pos(3, rowOffset, 30, 60));

add_line(subPath, [name '_bias_rw/1'], [name '_sum/2'], 'autorouting', 'on');
add_line(subPath, [name '_bias0/1'], [name '_sum/3'], 'autorouting', 'on');
add_line(subPath, [name '_meas_noise/1'], [name '_sum/4'], 'autorouting', 'on');
end

function set_chart_script(blockPath, script, sampleTime)
% Insert SCRIPT as the body of the MATLAB Function block at BLOCKPATH,
% and (if SAMPLETIME is given) force that block to run at a fixed
% discrete sample time instead of inheriting one - required for a block
% whose script uses a persistent variable. See the "Create MATLAB
% Function Block Programmatically" pattern in the MATLAB documentation
% (MATLAB Function blocks are implemented as Stateflow charts under the
% hood, and sample time is a property of that chart object - not a
% plain block parameter settable via set_param on the block itself).
try
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    chart.Script = script;
catch ME
    warning('build_model:chartScript', ...
        ['Could not set the code for %s automatically (%s).\n' ...
         'Open the model, double-click that block, and paste the code from ' ...
         'sim/blocks/ manually instead.'], blockPath, ME.message);
    return
end

if nargin >= 3 && ~isempty(sampleTime)
    try
        chart.SampleTime = num2str(sampleTime);
    catch ME2
        warning('build_model:chartSampleTime', ...
            ['Could not set the discrete sample time on %s automatically (%s).\n' ...
             'Open that block''s dialog (right-click > Block Parameters) and set ' ...
             '"Sample time" to %.10g manually - this block uses a persistent variable ' ...
             'and will error out if it inherits a continuous sample time.'], ...
            blockPath, ME2.message, sampleTime);
    end
end
end

function try_set_param(blk, varargin)
% set_param wrapper that warns (instead of erroring out and aborting the
% whole build) if a parameter name/value is not accepted - e.g. if the
% Band-Limited White Noise block's parameter names differ slightly in
% your MATLAB version. Open the block dialog and set Noise power /
% Sample time / Seed by hand if you see this warning.
try
    set_param(blk, varargin{:});
catch ME
    warning('build_model:setParam', ...
        ['Could not set parameters on %s (%s).\n' ...
         'Set them manually in the block dialog: Noise power / Sample time / Seed.'], ...
        blk, ME.message);
end
end

function p = pos(col, row, w, h)
% Simple grid layout helper: column/row -> [left top right bottom].
left = 30 + col * 180;
top  = 30 + row * 90;
p = [left, top, left + w, top + h];
end
