function truth = generate_trajectory(waypoints, v_cruise, dt, save_path)
%GENERATE_TRAJECTORY Build a ground-truth pose/control history for v0.
%   TRUTH = GENERATE_TRAJECTORY(WAYPOINTS, V_CRUISE, DT, SAVE_PATH) drives
%   a vehicle through WAYPOINTS (an N x 2 list of [x y] points, e.g. from
%   map/pick_route.m) at a single constant speed V_CRUISE, with a short
%   constant-rate in-place turn at every intermediate waypoint where the
%   heading changes. No acceleration/braking - speed is constant for the
%   whole run, only heading changes (accelerometer reads ~0 except
%   during a turn's centripetal blip; gyro reads the turn rate).
%
%   Because the "true" control input (forward acceleration a_true(t),
%   always 0 here, and yaw rate w_true(t)) is known analytically by
%   construction, the exact ground-truth pose is obtained by integrating
%   it with zero sensor error using the same RK4 integrator used for
%   dead reckoning (see unicycle_rk4_step.m) - this is what "precise/
%   exact coordinates" means here: not measured, but known by
%   construction.
%
%   Returns TRUTH with fields:
%     t                 - time vector [s]
%     a_true, w_true    - true forward accel [m/s^2] (always 0) and yaw
%                          rate [rad/s]
%     x, y, psi, v       - true pose (m, m, rad, m/s)
%     waypoints          - the input waypoints, for reference/plotting

if nargin < 2 || isempty(v_cruise), v_cruise = 8;    end   % m/s (~29 km/h)
if nargin < 3 || isempty(dt),       dt = 0.01;       end   % s
if nargin < 4 || isempty(save_path)
    save_path = fullfile(fileparts(mfilename('fullpath')), 'truth_trajectory.mat');
end

% Duration of a full 90-degree turn. Since speed is constant (no
% braking for corners), the vehicle keeps moving forward *while* it
% turns, covering some extra arc length during every heading change -
% so below, the turn's duration (and therefore its extra distance)
% scales with how sharp it actually is: a real 90-degree corner takes
% the full turn_time_90, but a 2-degree wobble (typical hand-drawn
% jitter from map/pick_route.m, which can produce dozens of waypoints
% along one route) takes almost none. Without this scaling, every one
% of those small wobbles would cost the same fixed time/distance as a
% real corner, and a hand-drawn route with many waypoints would end up
% far longer than the line you actually drew.
turn_time_90 = 0.5; % s, for a 90-degree turn

nseg        = size(waypoints, 1) - 1;
seg_vec     = diff(waypoints);
seg_len     = vecnorm(seg_vec, 2, 2);
seg_heading = atan2(seg_vec(:,2), seg_vec(:,1));

% ---- Build a piecewise timeline of (duration, a, w) blocks ----
dur = []; a_blk = []; w_blk = [];

for k = 1:nseg
    dur(end+1)   = seg_len(k) / v_cruise; %#ok<AGROW>
    a_blk(end+1) = 0;                     %#ok<AGROW>
    w_blk(end+1) = 0;                     %#ok<AGROW>

    if k < nseg
        dpsi = wrap_to_pi(seg_heading(k+1) - seg_heading(k));
        % Scale duration by |dpsi| (a 90-degree turn takes turn_time_90,
        % a tiny wobble takes proportionally less) - this keeps the
        % turn rate itself roughly constant regardless of how sharp the
        % corner is, which is closer to how a real vehicle turns anyway.
        this_turn_time = max(turn_time_90 * abs(dpsi) / (pi/2), dt);
        dur(end+1)   = this_turn_time;        %#ok<AGROW>
        a_blk(end+1) = 0;                     %#ok<AGROW>
        w_blk(end+1) = dpsi / this_turn_time; %#ok<AGROW>
    end
end

% ---- Sample (a_true, w_true) on a uniform time grid ----
% Build directly from an integer sample count per block, NOT a
% time-window mask (t >= tb & t < tb+dur(k)): a mask can silently drop
% an entire block if its window happens to fall between two grid
% points (a real risk for a very short block - e.g. a small heading
% wobble whose duration is clamped near one sample by the scaling
% above), because floating-point tb rarely lands exactly on a multiple
% of dt. Rounding each block to whole samples guarantees every block,
% however short, contributes at least one real sample - no gyro turn
% event can vanish from the signal.
n_samples = max(1, round(dur / dt));
a_parts = cell(1, numel(dur));
w_parts = cell(1, numel(dur));
for k = 1:numel(dur)
    a_parts{k} = repmat(a_blk(k), n_samples(k), 1);
    w_parts{k} = repmat(w_blk(k), n_samples(k), 1);
end
a_true = cat(1, a_parts{:});
w_true = cat(1, w_parts{:});
t = (0:numel(a_true))' * dt; % one more point than control samples (initial state + one per step)
% Pad a_true/w_true by one repeated sample so they're the same length
% as t - convenient for plotting them against t directly. The RK4 loop
% below only ever reads indices 1..numel(t)-1, so this padded last
% sample is never actually used for integration.
a_true(end+1) = a_true(end); %#ok<AGROW>
w_true(end+1) = w_true(end); %#ok<AGROW>

% ---- Integrate exactly (RK4, zero sensor error) for the true pose ----
% Starts already moving at v_cruise (constant speed for the whole run).
s0 = [waypoints(1,1); waypoints(1,2); seg_heading(1); v_cruise];
S = zeros(4, numel(t));
S(:,1) = s0;
for kk = 1:numel(t)-1
    S(:,kk+1) = unicycle_rk4_step(S(:,kk), a_true(kk), w_true(kk), dt);
end

truth.t         = t;
truth.a_true    = a_true;
truth.w_true    = w_true;
truth.x         = S(1,:)';
truth.y         = S(2,:)';
truth.psi       = S(3,:)';
truth.v         = S(4,:)';
truth.waypoints = waypoints;

save(save_path, 'truth');
end

function a = wrap_to_pi(a)
% Wrap an angle to (-pi, pi]. Implemented locally to avoid a dependency
% on wrapToPi, which lives in a toolbox that may not be licensed.
a = mod(a + pi, 2*pi) - pi;
end
