function [x, y, psi, v] = ins_mechanization_periodic(a_meas, w_meas, Ts, x0, y0, psi0, v0, nodes, edges, correction_period)
%#codegen
% INS Mechanization + Periodic Road Correction - MATLAB Function block.
%
% This is the exact code build_model.m tries to insert into the
% "INS Mechanization + Periodic Correction" block of
% gps_free_nav_v0.slx. If the automatic insertion fails, open the
% model, double-click that (empty) MATLAB Function block, and paste
% this whole file's contents in.
%
% Runs the SAME RK4 dead-reckoning integration as
% "INS Mechanization (RK4)" (sim/blocks/ins_mechanization_block.m), on
% its own independent state - starts out identical to that block's
% (never-corrected) output, but every CORRECTION_PERIOD seconds it
% snaps its own running position onto the nearest road (same
% nearest-segment-plus-hysteresis idea as
% sim/blocks/map_matching_block.m) and aligns its heading to that
% road's direction, so drift doesn't keep accumulating unchecked
% between corrections. This is the "green" trajectory: starts as the
% raw ("red") dead reckoning, periodically pulled back onto the map -
% the "Map Matching" block then does its per-sample matching on top of
% this (already road-aware) signal instead of on the raw one.

persistent s
persistent t_acc
persistent cur_edge
if isempty(s)
    s = [x0; y0; psi0; v0];
    t_acc = 0;
    cur_edge = 0;
end

s = rk4_step(s, a_meas, w_meas, Ts);
t_acc = t_acc + Ts;

if t_acc >= correction_period
    t_acc = 0;
    [xm, ym, e] = match_with_hysteresis(s(1), s(2), nodes, edges, cur_edge);
    cur_edge = e;
    a = nodes(edges(e,1), :);
    b = nodes(edges(e,2), :);
    s(1) = xm;
    s(2) = ym;
    s(3) = atan2(b(2) - a(2), b(1) - a(1));
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

function [xm, ym, edge_idx] = match_with_hysteresis(x, y, nodes, edges, cur_edge)
margin = 3; % m - how much closer another road must be before switching
n_edges = size(edges, 1);
best_dist = inf; best_x = x; best_y = y; best_edge = 0;
for e = 1:n_edges
    [px, py, d] = project_to_segment(x, y, nodes(edges(e,1),:), nodes(edges(e,2),:));
    if d < best_dist
        best_dist = d; best_x = px; best_y = py; best_edge = e;
    end
end
if cur_edge ~= 0
    [cx, cy, cd] = project_to_segment(x, y, nodes(edges(cur_edge,1),:), nodes(edges(cur_edge,2),:));
    if cd <= best_dist + margin
        best_edge = cur_edge; best_x = cx; best_y = cy;
    end
end
xm = best_x; ym = best_y; edge_idx = best_edge;
end

function [px, py, d] = project_to_segment(x, y, a, b)
ab = b - a;
denom = ab(1)^2 + ab(2)^2;
if denom < eps
    t = 0;
else
    t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom;
    t = min(max(t, 0), 1);
end
px = a(1) + t*ab(1);
py = a(2) + t*ab(2);
d = hypot(x - px, y - py);
end
