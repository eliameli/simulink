function [x, y, psi, v] = ins_mechanization(a_meas, w_meas, Ts, x0, y0, psi0, v0, nodes, edges)
%#codegen
% INS Mechanization (RK4) - MATLAB Function block.
%
% This is the exact code build_model.m tries to insert into the
% "INS Mechanization (RK4)" block of gps_free_nav_v0.slx. If the
% automatic insertion fails, open the model, double-click that (empty)
% MATLAB Function block, and paste this whole file's contents in.
%
% Dead-reckoning: integrates the noisy sensor measurements (a_meas,
% w_meas) forward in time with RK4 (not Euler) to keep a running
% estimate of the vehicle pose [x; y; psi; v]. Ts is the fixed sample
% time (must match the model's fixed-step size). x0,y0,psi0,v0 is the
% known starting pose - dead reckoning always needs *some* known
% starting point, so this comes in from outside (set by run_v0.m from
% the ground-truth trajectory's first sample) rather than being
% hardcoded, since the route (and therefore the start point) changes
% every time you click a new one with map/pick_route.m.
%
% Corner snap: a real turn shows up as a clear, large spike in the
% measured yaw rate (tens-hundreds of deg/s by construction - see
% truth/generate_trajectory.m), easily told apart from ordinary gyro
% noise/bias (well under 1 deg/s). When a turn is confirmed over, the
% running position is pulled (partially, not teleported outright)
% toward the nearest map node, and heading is reset to whichever road
% leaving that node best matches the current heading. CAPTURE_RADIUS
% used to cap how far away a node could still be and count (so a
% heavily-drifted estimate couldn't "cheat" by jumping a long
% distance) - but that meant once drift ever exceeded it, no future
% turn could ever correct it again, which defeats the point: it's now
% infinite, so a corner-snap always finds and pulls toward the nearest
% node, however far off that is.
%
% An earlier, simpler version of this (bare threshold, no debounce/
% cooldown) mis-fired repeatedly on a single real corner whenever the
% drawn route's heading changed in several small steps close together
% (typical hand-drawn jitter - see map/pick_route.m), each one its own
% brief yaw-rate spike: every one of those triggered its own snap,
% producing a scribble of repeated corrections instead of one clean
% one. Fixed here with:
%   - hysteresis (separate enter/exit rate thresholds, so noise
%     sitting near one threshold can't flip the state back and forth),
%   - debounce (the rate has to stay above/below threshold for
%     DEBOUNCE_TIME seconds before a transition counts, so a handful of
%     noisy samples can't fake a whole turn), and
%   - a cooldown (COOLDOWN_S) between corrections, so even if several
%     micro-turns from one wobbly corner do each get detected, only the
%     first one actually triggers a snap.
% Verified against a standalone Python re-implementation of this exact
% state machine on synthetic data before writing this: a single clean
% 90-degree turn and an equivalent "wobbly" corner built from four
% smaller turns a few hundredths of a second apart both produced
% exactly one correction, landing on the same node/heading; a turn far
% from any map node correctly produced zero corrections.
%
% Startup warm-up: the corner-snap above only ever fires once a turn
% has been detected and confirmed, so for a route that starts with a
% long straight stretch there's nothing to correct against yet - and
% ordinary sensor bias/noise can already carry the estimate visibly off
% the road before the very first turn even happens. For the first
% WARMUP_TIME seconds, skip turn detection and instead just continuously
% project the running position onto the nearest road (like the "Map
% Matching" block does, but every step, not just for display) and align
% heading to that road's direction - the same idea as the corner snap,
% just running continuously instead of waiting for a turn.
%
% A first version of this picked whichever road segment the position
% was closest to, full stop - which mis-fired right at the very start
% whenever the route began at a "dead end" node (one direction removed
% by map/generate_map.m, so only e.g. "straight ahead" and "turn"
% actually exist there) and the drawn/clicked starting point wasn't
% exactly on the intended road's line (it never precisely is, by hand).
% The intended road doesn't extend backward past its own start node, so
% its nearest point clamps to that node and can end up geometrically
% farther away than the *other* road leaving the same node - even
% though the current heading clearly says which one was meant. Checked
% against several such near-node starting offsets in a standalone
% Python re-implementation: plain nearest-distance picked the wrong
% road every time, while breaking ties (within TIE_TOL) by which
% direction better matches the current heading picked the right one
% every time. Hysteresis (persistent WARMUP_EDGE) on top keeps that
% choice from flip-flopping step to step once it's made.

persistent s
persistent t_elapsed
persistent warmup_edge
persistent high_time
persistent low_time
persistent rate_state
persistent since_correction
if isempty(s)
    s = [x0; y0; psi0; v0];
    t_elapsed = 0;
    warmup_edge = 0;
    high_time = 0;
    low_time = 0;
    rate_state = 0;       % 0 = confirmed straight, 1 = confirmed turning
    since_correction = 1; % seconds since the last correction (>= cooldown: eligible right away)
end

s = rk4_step(s, a_meas, w_meas, Ts);
t_elapsed = t_elapsed + Ts;
since_correction = since_correction + Ts;

warmup_time     = 3.0;  % s, see note above
rate_threshold_high = 0.35; % rad/s (~20 deg/s) - enter "turning"
rate_threshold_low  = 0.17; % rad/s (~10 deg/s) - exit "turning" (hysteresis gap vs. the enter threshold)
debounce_time   = 0.05; % s, rate must stay past a threshold this long to count
cooldown_s      = 1.0;  % s, minimum time between corrections
capture_radius  = inf;  % m, always correct to the nearest node, however far off it's drifted
position_blend  = 0.7;  % 0..1, how much of the way to pull toward the node (not a hard teleport)

if t_elapsed <= warmup_time
    [px, py, edir, warmup_edge] = nearest_edge_point(s(1), s(2), s(3), nodes, edges, warmup_edge);
    s(1) = px;
    s(2) = py;
    s(3) = pick_heading(edir, s(3));
else
    if abs(w_meas) > rate_threshold_high
        high_time = high_time + Ts;
        low_time = 0;
    elseif abs(w_meas) < rate_threshold_low
        low_time = low_time + Ts;
        high_time = 0;
    else
        high_time = 0;
        low_time = 0;
    end

    if rate_state == 0 && high_time >= debounce_time
        rate_state = 1;
    end

    if rate_state == 1 && low_time >= debounce_time
        rate_state = 0;
        if since_correction >= cooldown_s
            [nx, ny, npsi, found] = try_snap(s(1), s(2), s(3), nodes, edges, capture_radius);
            if found
                s(1) = s(1) + position_blend * (nx - s(1));
                s(2) = s(2) + position_blend * (ny - s(2));
                s(3) = npsi;
                since_correction = 0;
            end
        end
    end
end

x = s(1); y = s(2); psi = s(3); v = s(4);
end

function s_next = rk4_step(s, a, w, dt)
k1 = deriv(s,          a, w);
k2 = deriv(s + dt/2*k1, a, w);
k3 = deriv(s + dt/2*k2, a, w);
k4 = deriv(s + dt*k3,   a, w);
s_next = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

function ds = deriv(s, a, w)
psi = s(3); v = s(4);
ds = [v*cos(psi); v*sin(psi); w; a];
end

function [nx, ny, npsi, found] = try_snap(x, y, psi, nodes, edges, capture_radius)
% Nearest map node, and (if it's close enough) the heading of whichever
% road leaving that node best matches the current heading.
d2 = (nodes(:,1) - x).^2 + (nodes(:,2) - y).^2;
[dmin2, i] = min(d2);
if sqrt(dmin2) > capture_radius
    nx = x; ny = y; npsi = psi; found = false;
    return
end

nx = nodes(i,1);
ny = nodes(i,2);
npsi = psi;
best_diff = inf;
for e = 1:size(edges, 1)
    if edges(e,1) == i
        other = edges(e,2);
    elseif edges(e,2) == i
        other = edges(e,1);
    else
        continue
    end
    dir = atan2(nodes(other,2) - ny, nodes(other,1) - nx);
    diff = abs(wrap_to_pi(dir - psi));
    if diff < best_diff
        best_diff = diff;
        npsi = dir;
    end
end
found = true;
end

function a = wrap_to_pi(a)
a = mod(a + pi, 2*pi) - pi;
end

function [px, py, edir, edge_idx] = nearest_edge_point(x, y, psi, nodes, edges, cur_edge)
% Closest point on a road segment, that segment's direction, and which
% segment was picked - same nearest-point-on-segment idea as
% sim/blocks/map_matching_block.m, but with two extra safeguards
% needed at the very start (see the note above where this is called):
%   - among all segments within TIE_TOL of the closest one, pick
%     whichever direction best matches the current heading, instead of
%     blindly trusting raw distance (which is ambiguous right where a
%     road starts, since its nearest point can't extend past that
%     start node);
%   - hysteresis (MARGIN) against CUR_EDGE, so the choice doesn't flip
%     between two similarly-close segments from one step to the next.
tie_tol = 5; % m
margin  = 3; % m

n = size(edges, 1);
proj_x = zeros(n, 1); proj_y = zeros(n, 1);
proj_d = zeros(n, 1); proj_dir = zeros(n, 1);
best_dist = inf;

for e = 1:n
    a = nodes(edges(e,1), :);
    b = nodes(edges(e,2), :);
    ab = b - a;
    denom = ab(1)^2 + ab(2)^2;
    if denom < eps
        t = 0;
    else
        t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom;
        t = min(max(t, 0), 1);
    end
    proj = a + t*ab;
    d = hypot(x - proj(1), y - proj(2));
    proj_x(e) = proj(1); proj_y(e) = proj(2);
    proj_d(e) = d; proj_dir(e) = atan2(ab(2), ab(1));
    if d < best_dist
        best_dist = d;
    end
end

best_e = 1;
best_hcost = inf;
for e = 1:n
    if proj_d(e) <= best_dist + tie_tol
        hcost = min(abs(wrap_to_pi(proj_dir(e) - psi)), abs(wrap_to_pi(proj_dir(e) + pi - psi)));
        if hcost < best_hcost
            best_hcost = hcost;
            best_e = e;
        end
    end
end

if cur_edge ~= 0 && proj_d(cur_edge) <= best_dist + margin
    best_e = cur_edge;
end

px = proj_x(best_e); py = proj_y(best_e); edir = proj_dir(best_e); edge_idx = best_e;
end

function h = pick_heading(edir, current_psi)
% A road segment's direction is ambiguous by 180 degrees (it doesn't
% know which way you're driving on it) - pick whichever of the two
% matches the current heading more closely, so warm-up doesn't flip
% the direction of travel.
cand1 = edir;
cand2 = wrap_to_pi(edir + pi);
if abs(wrap_to_pi(cand1 - current_psi)) <= abs(wrap_to_pi(cand2 - current_psi))
    h = cand1;
else
    h = cand2;
end
end
