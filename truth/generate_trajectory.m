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

% Duration of each heading change at an intermediate waypoint. Since
% speed is constant (no braking for corners), the vehicle keeps moving
% forward *while* it turns, covering v_cruise*turn_time meters of arc
% during the turn instead of pivoting truly in place - keep this short,
% or corners on a route with short segments (typical when you click a
% route close together) visibly bulge outward past where you clicked.
turn_time = 0.5; % s

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
        dur(end+1)   = turn_time;        %#ok<AGROW>
        a_blk(end+1) = 0;                %#ok<AGROW>
        w_blk(end+1) = dpsi / turn_time; %#ok<AGROW>
    end
end

% ---- Sample (a_true, w_true) on a uniform time grid ----
total_t = sum(dur);
t = (0:dt:total_t)';
a_true = zeros(size(t));
w_true = zeros(size(t));
tb = 0;
for k = 1:numel(dur)
    mask = t >= tb & t < tb + dur(k);
    a_true(mask) = a_blk(k);
    w_true(mask) = w_blk(k);
    tb = tb + dur(k);
end

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
