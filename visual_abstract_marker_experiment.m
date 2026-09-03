% VISUAL_ABSTRACT_MARKER_EXPERIMENT
% Generates a single visual-abstract figure from two real PRM searches over
% the same sampled roadmap. Path A is prediction-only (marker unavailable).
% Path B is marker-aware and may detour to the stigmergic sensing region.

clear
close all
clc

%% Reproducible experiment parameters
rng(23, 'twister')

N = 2000;
connection_radius = 0.5;
lambda = 2500;            % Emphasizes terminal uncertainty for the visual.
F = eye(2);
R = 1e-4 * eye(2);        % Process-noise covariance per traveled meter.
robot_speed = 1.0;
chi = chi2inv(0.8, 2);
num_props = 8;

bound(1).x = [0, 1];
bound(2).x = [0, 1];
start = [0.10, 0.50];
target = [0.86, 0.96; 0.44, 0.56];

% Five polygonal obstacles, specified counterclockwise by vertices.
obstacles(1).x = [0.20 0.43 0.40 0.23];
obstacles(1).y = [0.76 0.80 0.60 0.61];
obstacles(2).x = [0.18 0.34 0.39 0.22];
obstacles(2).y = [0.40 0.47 0.26 0.21];
obstacles(3).x = [0.65 0.64 0.76 0.75];
obstacles(3).y = [0.40 0.20 0.25 0.45];
obstacles(4).x = [0.62 0.80 0.81 0.66 0.59];
obstacles(4).y = [0.75 0.80 0.63 0.60 0.69];
obstacles(5).x = [0.40 0.60 0.50];
obstacles(5).y = [0.50 0.45 0.60];
obstacle_edge = polygons_to_edges(obstacles);

marker.x = [0.87, 0.13];
marker.sensing_radius = 0.10;
marker.H = eye(2);
marker.R = 1e-5 * eye(2);

%% Construct one shared spatial PRM
positions = zeros(N, 2);
positions(1, :) = start;
for ii = 2:N
    positions(ii, :) = sample_free_position_polygons(bound, obstacles);
end
neighbor_ID = rangesearch(positions, positions, connection_radius);

node_template = initialize_nodes(positions, 1e-4 * eye(2), lambda, chi);
prop(1:num_props) = struct('x', [-100 -100], 'P', -100 * eye(2), ...
    'ra', -100, 'rb', -100, 'ang', -100, 'ellipse_rect', zeros(1, 4));

%% Path A: prediction-only PRM
node_off = node_template;
[node_off, distance_off, predecessor_off] = dijkstra_ekf_prm(node_off, ...
    neighbor_ID, F, R, robot_speed, lambda, obstacle_edge, chi, bound, ...
    num_props, prop);
path_off = select_target_path(node_off, distance_off, predecessor_off, target);

%% Path B: marker-aware PRM over the identical sampled positions
node_on = node_template;
[node_on, ~, ~, path_on, ~] = dijkstra_ekf_marker_prm(node_on, neighbor_ID, ...
    F, R, robot_speed, lambda, marker, obstacle_edge, chi, bound, ...
    num_props, prop, target);

if isempty(path_off) || isempty(path_on)
    error('visual_abstract_marker_experiment:NoPath', ...
        ['The visual-abstract configuration did not produce both paths ', ...
         '(Path A empty: %d; Path B empty: %d). Increase N or ', ...
         'connection_radius and rerun.'], isempty(path_off), isempty(path_on))
end

metrics_off = calculate_path_metrics(node_off, path_off, lambda);
metrics_on = calculate_path_metrics(node_on, path_on, lambda);

%% Plot both planner outputs on one axis
path_a_color = [0.80 0.16 0.14];
path_b_color = [0.00 0.35 0.80];
marker_color = [0.4940 0.1840 0.5560];

fig_f = figure('Color', 'w');
fig_f.Position = [150 90 760 710];
ax = axes(fig_f);
hold(ax, 'on')
axis(ax, 'equal')
xlim(ax, bound(1).x)
ylim(ax, bound(2).x)
grid(ax, 'on')
box(ax, 'on')
set(ax, 'FontName', 'Arial', 'FontSize', 12, 'LineWidth', 1.2)

for ii = 1:numel(obstacles)
    fill(ax, obstacles(ii).x, obstacles(ii).y, [0.63 0.65 0.66], ...
        'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 1)
end

rectangle(ax, 'Position', [target(1, 1), target(2, 1), ...
    diff(target(1, :)), diff(target(2, :))], ...
    'FaceColor', [0.80 1.00 0.80], 'EdgeColor', [0.0 0.5 0.0], ...
    'LineWidth', 1.5);

theta = linspace(0, 2*pi, 160);
range_handle = plot(ax, marker.x(1) + marker.sensing_radius*cos(theta), ...
    marker.x(2) + marker.sensing_radius*sin(theta), '--', ...
    'Color', marker_color, 'LineWidth', 1.5);
marker_handle = plot(ax, marker.x(1), marker.x(2), 'p', ...
    'Color', marker_color, 'MarkerFaceColor', marker_color, 'MarkerSize', 13);

% Draw Path A first, then Path B on top so both routes remain visible.
path_a_handle = plot_path_and_covariance(ax, node_off, path_off, R, chi, ...
    num_props, [], false, path_a_color);
path_b_handle = plot_path_and_covariance(ax, node_on, path_on, R, chi, ...
    num_props, marker, true, path_b_color);

start_handle = plot(ax, start(1), start(2), 'o', 'Color', [0.85 0 0], ...
    'MarkerFaceColor', [0.85 0 0], 'MarkerSize', 8);
goal_handle = plot(ax, node_on(path_on(end)).x(1), ...
    node_on(path_on(end)).x(2), 'o', 'Color', [0 0.5 0], ...
    'MarkerFaceColor', [0 0.5 0], 'MarkerSize', 8);

xlabel(ax, 'Location X [m]')
ylabel(ax, 'Location Y [m]')
title(ax, 'Marker Visibility-Aware Belief-Space PRM', ...
    'FontWeight', 'bold', 'FontSize', 16)

legend(ax, [path_a_handle, path_b_handle, range_handle, marker_handle, ...
    start_handle, goal_handle], ...
    {'Path A trajectory and covariance', 'Path B trajectory and covariance', ...
     'Marker sensing range', 'Marker', 'Start', 'Goal'}, ...
    'Location', 'southwest', 'FontSize', 9)

exportgraphics(fig_f, 'data/visual_abstract_marker_prm.png', ...
    'Resolution', 300)

fprintf('Path A (marker unavailable): Dtravel = %.6f m, tr(Pgoal) = %.6e, Dtotal = %.6f\n', ...
    metrics_off.travel, metrics_off.trace_goal, metrics_off.total)
fprintf('Path B (marker-aware)     : Dtravel = %.6f m, tr(Pgoal) = %.6e, Dtotal = %.6f\n', ...
    metrics_on.travel, metrics_on.trace_goal, metrics_on.total)

function node = initialize_nodes(positions, P0, lambda, chi)
N = size(positions, 1);
initial = struct('x', [-100 -100], 'P', -100 * eye(2), 't', -100, ...
    'parent', 0, 'value', inf, 'ra', -100, 'rb', -100, 'ang', -100, ...
    'ellipse_rect', [-100 -100 -100 -100]);
node(1:N) = initial;
for ii = 1:N
    node(ii).x = positions(ii, :);
end
node(1).P = P0;
node(1).t = 0;
node(1).value = lambda * trace(P0);
[node(1).ra, node(1).rb, node(1).ang, node(1).ellipse_rect] = ...
    error_ellipse(node(1).x, node(1).P, chi);
end

function x = sample_free_position_polygons(bound, obstacles)
while true
    x = [bound(1).x(1) + diff(bound(1).x) * rand, ...
         bound(2).x(1) + diff(bound(2).x) * rand];
    inside = false;
    for ii = 1:numel(obstacles)
        if inpolygon(x(1), x(2), obstacles(ii).x, obstacles(ii).y)
            inside = true;
            break
        end
    end
    if ~inside
        return
    end
end
end

function edges = polygons_to_edges(obstacles)
edges = struct('start', {}, 'end', {}, 'slope', {}, 'y_inter', {});
for ii = 1:numel(obstacles)
    vertices = [obstacles(ii).x(:), obstacles(ii).y(:)];
    for jj = 1:size(vertices, 1)
        next = mod(jj, size(vertices, 1)) + 1;
        edges(end+1).start = vertices(jj, :); %#ok<AGROW>
        edges(end).end = vertices(next, :);
        dx = edges(end).start(1) - edges(end).end(1);
        dy = edges(end).start(2) - edges(end).end(2);
        edges(end).slope = dy / dx;
        edges(end).y_inter = edges(end).start(2) - edges(end).slope * edges(end).start(1);
    end
end
end

function path = select_target_path(node, distance, predecessor, target)
N = numel(node);
all_rectangles = reshape([node.ellipse_rect], 4, N).';
in_target = all_rectangles(:, 1) >= target(1, 1) & ...
    all_rectangles(:, 1) + all_rectangles(:, 3) <= target(1, 2) & ...
    all_rectangles(:, 2) >= target(2, 1) & ...
    all_rectangles(:, 2) + all_rectangles(:, 4) <= target(2, 2);
target_ID = find(in_target & isfinite(distance));
path = [];
if ~isempty(target_ID)
    [~, relative] = min(distance(target_ID));
    path = reconstruct_prm_path(predecessor, target_ID(relative), 1);
end
end

function metrics = calculate_path_metrics(node, path, lambda)
positions = reshape([node(path).x], 2, []).';
metrics.travel = sum(vecnorm(diff(positions, 1, 1), 2, 2));
metrics.trace_goal = trace(node(path(end)).P);
metrics.total = metrics.travel + lambda * metrics.trace_goal;
end

function trajectory_handle = plot_path_and_covariance(ax, node, path, R, chi, nprop, marker, marker_enabled, color)
marker_has_been_sensed = false;
for ii = 1:numel(path)-1
    x0 = node(path(ii)).x;
    x1 = node(path(ii+1)).x;
    P0 = node(path(ii)).P;
    d = norm(x1 - x0);
    sensed_edge = false;
    if marker_enabled && ~marker_has_been_sensed
        [sensed_edge, eta] = marker_encounter(x0, x1, marker);
        if sensed_edge
            P_pre = P0 + R * (eta * d);
            P_post = ekf_update_covariance(P_pre, marker.H, marker.R);
        end
    end
    for jj = 0:nprop
        fraction = jj / nprop;
        x = x0 + fraction * (x1 - x0);
        if sensed_edge && fraction > eta
            P = P_post + R * ((fraction - eta) * d);
        else
            P = P0 + R * (fraction * d);
        end
        draw_covariance_ellipse(ax, x, P, chi, color)
    end
    if sensed_edge
        marker_has_been_sensed = true;
    end
end
positions = reshape([node(path).x], 2, []).';
trajectory_handle = plot(ax, positions(:, 1), positions(:, 2), '-', ...
    'Color', color, 'LineWidth', 2.2);
plot(ax, positions(:, 1), positions(:, 2), 'o', 'Color', color, ...
    'MarkerFaceColor', color, 'MarkerSize', 3.5)
end

function draw_covariance_ellipse(ax, x, P, chi, color)
[ra, rb, angle] = error_ellipse(x, P, chi);
theta = linspace(0, 2*pi, 80);
ellipse = [ra * cos(theta); rb * sin(theta)];
rotation = [cos(angle), sin(angle); -sin(angle), cos(angle)];
ellipse = rotation * ellipse;
plot(ax, ellipse(1, :) + x(1), ellipse(2, :) + x(2), '-', ...
    'Color', color, 'LineWidth', 0.65)
end
