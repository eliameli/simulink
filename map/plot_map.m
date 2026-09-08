function plot_map(map)
%PLOT_MAP Draw the road network onto the current axes.
%   Call this with HOLD already ON if you want to overlay trajectories.
%   Road segments and node markers are excluded from the legend
%   (HandleVisibility off) so they don't clutter it.

for e = 1:size(map.edges, 1)
    a = map.nodes(map.edges(e,1), :);
    b = map.nodes(map.edges(e,2), :);
    plot([a(1) b(1)], [a(2) b(2)], 'Color', [0.6 0.6 0.6], 'LineWidth', 10, ...
        'HandleVisibility', 'off');
end
plot(map.nodes(:,1), map.nodes(:,2), 'ks', 'MarkerFaceColor', [0.6 0.6 0.6], ...
    'HandleVisibility', 'off');
end
