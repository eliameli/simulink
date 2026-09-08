function [x, y, psi, v] = ins_mechanization(a_meas, w_meas, Ts, x0, y0, psi0, v0)
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

persistent s
if isempty(s)
    s = [x0; y0; psi0; v0];
end

s = rk4_step(s, a_meas, w_meas, Ts);

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
