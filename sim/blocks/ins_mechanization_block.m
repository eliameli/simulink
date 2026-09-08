function [x, y, psi, v] = ins_mechanization(a_meas, w_meas, Ts, x0, y0, psi0, v0, nodes)
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
% Corner snap: every real turn shows up as a clear spike in the
% measured yaw rate (tens to hundreds of deg/s - see
% truth/generate_trajectory.m, turns are built to have a roughly
% constant, large rate), easily told apart from ordinary gyro
% noise/bias (well under 1 deg/s). Right when a turn ends (the
% measured rate drops back below TURN_RATE_THRESHOLD after being
% above it), snap the running position onto the nearest map
% intersection (node) and keep integrating from there - this stops
% position error from one corner compounding into the next, without
% needing a separate corrected trajectory.

persistent s
persistent in_turn
if isempty(s)
    s = [x0; y0; psi0; v0];
    in_turn = false;
end

s = rk4_step(s, a_meas, w_meas, Ts);

turn_rate_threshold = 0.35; % rad/s (~20 deg/s) - above sensor noise/bias, below a real turn's rate
was_in_turn = in_turn;
in_turn = abs(w_meas) > turn_rate_threshold;

if was_in_turn && ~in_turn
    [nx, ny] = nearest_node(s(1), s(2), nodes);
    s(1) = nx;
    s(2) = ny;
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

function [nx, ny] = nearest_node(x, y, nodes)
d2 = (nodes(:,1) - x).^2 + (nodes(:,2) - y).^2;
[~, i] = min(d2);
nx = nodes(i,1);
ny = nodes(i,2);
end
