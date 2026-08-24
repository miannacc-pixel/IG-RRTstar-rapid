clear all
close all
clc

N = 10000; % Total number of roadmap vertices, including the initial vertex

%% Definition of the roadmap vertex structure

% Just setting initial values for unsampled vertices
ini_st = -100;
ini_P = diag([-100 5]);
ini_value = inf;

% The fields below are retained from the IG-RRT* implementation so that the
% existing plotting code can still display the final belief path.
node(1:N) = struct('x', ini_st*ones(1,2), 'P', ini_P, 'parent', 0,...
    'value', ini_value, 'ra', ini_st, 'rb', ini_st, 'ang', ini_st,...
    'ellipse_rect', ini_st*ones(1,4));

% node.x: The position (2-D) of the roadmap vertex
% node.P: The covariance (2-D) of the roadmap vertex
% node.parent: The predecessor on the final shortest path tree
% node.value: The shortest path cost from the initial vertex
% node.ra, node.rb, node.ang, node.ellipse_rect: Ellipse data for plotting

%% PRM parameters

% Two vertices are considered for a connection when their symmetric
% Euclidean-plus-Frobenius distance is less than this value. This is the
% same proxy distance used by Algorithm 2 for nearest-neighbor operations.
connection_radius = 0.15;

% The sampling routine normally uses these values to scale an RRT sample
% toward its nearest tree vertex. An infinite radius disables that scaling,
% yielding independent PRM samples while retaining the original collision-free
% belief-state sampler.
% sample_polyshape_check initializes its legacy nearest-vertex output only
% when this threshold is positive. realmin preserves independent PRM samples
% while allowing that initialization to occur.
radius_min = realmin;
sampling_radius = inf;

% Weight on information cost
alpha = 0.2;

% The gain of noise (In the paper, we denote this as W)
R = (1/1000)*eye(2);

% Confidence bound used for collision checking
chi = chi2inv(0.8,2);

%% Environment definition and properties

% Current environment is the "multiple obstacle" environment.
obstacle_edge = obstacle_multi();
obs_polyshape = obstacle_polyshape();

% Target(final) area [xmin, xmax; ymin, ymax]
target = [0.8, 0.9; 0.1, 0.2];

% Path planning area
bound(1).x = [0,1];
bound(2).x = [0,1];

% Acceptable range for the eigenvalues of the sampled covariance matrix
bound(1).P = [10^-9,10^-3];
bound(2).P = [10^-9,10^-3];

% Initial roadmap vertex
node(1).x = [0.1, 0.1];
node(1).P = 1e-4 * eye(2);
node(1).value = 0;
[node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = ...
    error_ellipse(node(1).x, node(1).P, chi);

%% Parameters for collision checking along a roadmap edge

% How many intermediate ellipses are used to check one directed edge
num_props = 10;
prop(1:num_props) = struct('x', ini_st*ones(1,2), 'P', ini_st*eye(2), ...
    'ra', ini_st, 'rb', ini_st, 'ang', ini_st, 'ellipse_rect', zeros(1,4));

%% PRM algorithm

tic

% Step 1: Sample collision-free belief states to create the roadmap vertices.
% No vertex is selected as a parent during sampling.
for ii = 2:N
    [x, P, ra, rb, ang, ellipse_rect] = sample_x_P_randomly(...
        bound, node(1), 1, radius_min, sampling_radius, chi, ...
        obstacle_edge, obs_polyshape);

    node(ii).x = x;
    node(ii).P = P;
    node(ii).ra = ra;
    node(ii).rb = rb;
    node(ii).ang = ang;
    node(ii).ellipse_rect = ellipse_rect;

end

% Step 2: Build a directed roadmap. The information-geometric cost is
% asymmetric, so feasibility and cost must be evaluated in both directions.
% A PRM vertex is a fixed belief state; therefore, only lossless transitions
% are added rather than modifying a vertex covariance as RRT* does.
edge_cost = inf(N,N);

for ii = 1:N-1
    for jj = ii+1:N
        proxy_distance = norm(node(ii).x - node(jj).x) + ...
            norm(node(ii).P - node(jj).P, 'fro');

        if proxy_distance <= connection_radius
            % Test the directed edge ii -> jj.
            if ~check_lossless(node(ii).x, node(ii).P, node(jj).x, node(jj).P, R)
                issue_flag = psuedo_obs_check_line2_oct(node(ii), node(jj), ...
                    obstacle_edge, R, chi, bound, num_props, prop);
                if ~issue_flag
                    edge_cost(ii,jj) = dist_ig_mat(node(ii).x.', node(ii).P, ...
                        node(jj).x.', node(jj).P, alpha, R);
                end
            end

            % Test the reverse directed edge jj -> ii independently.
            if ~check_lossless(node(jj).x, node(jj).P, node(ii).x, node(ii).P, R)
                issue_flag = psuedo_obs_check_line2_oct(node(jj), node(ii), ...
                    obstacle_edge, R, chi, bound, num_props, prop);
                if ~issue_flag
                    edge_cost(jj,ii) = dist_ig_mat(node(jj).x.', node(jj).P, ...
                        node(ii).x.', node(ii).P, alpha, R);
                end
            end
        end
    end
end

% Step 3: Search the completed directed roadmap from the initial vertex.
% Dijkstra's algorithm is valid because every travel-plus-information cost is
% nonnegative.
[distance, predecessor] = dijkstra_prm(edge_cost, 1);
for ii = 1:N
    node(ii).value = distance(ii);
    node(ii).parent = predecessor(ii);
end

% Find roadmap vertices whose complete confidence ellipses lie in the target.
all_rec = reshape([node.ellipse_rect], [4, N]).';
in_target = all_rec(:,1) >= target(1,1) & ...
    all_rec(:,1) + all_rec(:,3) <= target(1,2) & ...
    all_rec(:,2) >= target(2,1) & ...
    all_rec(:,2) + all_rec(:,4) <= target(2,2);
target_ID = find(in_target & isfinite(distance));

% Reconstruct the lowest-cost path from a target vertex back to vertex 1.
path = [];
min_path_leng = -10^10;
if ~isempty(target_ID)
    [min_path_leng, target_index] = min(distance(target_ID));
    path = reconstruct_prm_path(predecessor, target_ID(target_index), 1);
end

% Preserve the original save variables used by the plotting scripts.
saver = struct('path', path, 'node', N);
min_path_data = nan(N,1);
min_path_data(N) = min_path_leng;

% The file name used for saved data. Data is saved in the data folder.
savename = ['data/PRM_N', num2str(N), '_alpha_', num2str(alpha), ...
    '_radius_', num2str(connection_radius)];
savename(savename=='.') = [];
save(savename)

timeElapsed = toc;
