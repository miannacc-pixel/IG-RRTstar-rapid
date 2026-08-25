clear all
close all
clc

N = 2000; % Total number of roadmap vertices, including the initial vertex

%% Definition of the roadmap vertex structure

% Just setting initial values for unsampled vertices 
ini_st = -100; % initial value for unsampled nodes
ini_P = diag([-100 5]); % initial covariance for unsampled nodes
ini_value = 5000; % initial cost for unsampled nodes

% Initilize node with the value which will not be outputted by algorithm
node(1:N) = struct('x', ini_st*ones(1,2), 'P', ini_P, 'parent', 0,...
    'value', ini_value, 'ra', ini_st, 'rb', ini_st, 'ang', ini_st,...
    'ellipse_rect', ini_st*ones(1,4));

% node.x: The position (2-D) of the roadmap vertex   (1*2 vector)
% node.P: The covariance (2-D) of the roadmap vertex (2*2 matrix)
% node.parent: The predecessor on the final shortest path tree (scalar value)
% node.value: The shortest path cost from the initial vertex (scalar value)
% node.ra: The length of major axis of ellipse  (scalar value)
% node.rb: The length of minor axis of ellipse  (scalar value)
% node.ang: The rotation angle of the ellipse (range is from 0 to 2*pi) (scalar value)
% node.ellipse_rect: A bounding box which surrouds the ellipse 
%        [bottom-left-x bottom-left-y width height]  ([1, 4] matrix)

%% PRM parameters

% Two spatial PRM vertices are considered for a connection when their
% Euclidean distance is less than this value.
connection_radius = 0.5;

% Weight on information cost
alpha = 0.2;

% EKF prediction model. P is propagated along each accepted PRM edge rather
% than independently sampled at every roadmap vertex.
F = eye(2);

% The gain of process noise per traveled meter (W in the original paper).
R = (1/10000)*eye(2);

% Confidence bound used for collision checking
chi = chi2inv(0.8,2);

%% Environment definition and Properties

% current enviroment is  " multiple obstacle enviroment" 
        
        % define obstacle as a set of edges 
        % each edge is defined by: start point, end point, slope, and Y_axis
        % intercept
        obstacle_edge = obstacle_multi();
        obs_polyshape= obstacle_polyshape(); %definition of obstacles to use polyshape functionalities of Matlab

        % Target(final) area [xmin, xmax; ymin, ymax]
        target = [0.8, 0.9; 0.1, 0.2];

        % Path planning area
        bound(1).x = [0,1];
        bound(2).x = [0,1];

        % The position of the initial node
        node(1).x = [0.1, 0.1];

%% The setting for initial node
node(1).P = 1e-4 * eye(2);
node(1).value = 0;

[node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = error_ellipse(node(1).x, node(1).P, chi);
% [ra=major axis, rb=minor axis, ang= rotation angle , rect=bounding box] 
% = error_ellipse(x= 2D position of the ellipse, P = covariance , chi= confidence level)

%% Parameters for collision checking along a roadmap edge

% How many intermediate ellipses are used to check one directed edge
num_props = 10;

% Definition of intermediate ellipses 
prop(1:num_props) = struct('x', ini_st*ones(1,2), 'P', ini_st*eye(2), 'ra', ini_st,...
    'rb', ini_st, 'ang', ini_st, 'ellipse_rect', zeros(1,4));

% Initialize "prop", which have following structure
% prop.x: The position (2-D) of the node  (1*2 vector)
% prop.P: The covariance (2-D) of the node  (2*2 matrix)
% prop.ra: The length of major axis of ellipse  (scalar)
% prop.rb: The length of minor axis of ellipse  (scalar)
% prop.ang: The rotation angle of the ellipse   (range is from 0 to 2*pi)
% prop.ellipse_rect: A bounding box which surrouds the ellipse
%        [bottom-left-x bottom-left-y width height]
% prop.ellipse_rect: Used for collision checking (Boolean)

%% PRM algorithm

tic

% Step 1: Sample collision-free spatial states to create the roadmap vertices.
% Their covariances are assigned later by EKF propagation during the search.
for ii = 2:N
    node(ii).x = sample_free_position(bound, obs_polyshape);
end

% Step 2: Build spatial PRM neighborhoods. Edge covariances cannot be fixed
% here because they depend on the belief propagated to the source vertex.
x_all = reshape([node.x], 2, N).';
neighbor_ID = rangesearch(x_all, x_all, connection_radius);

% Step 3: Dijkstra search with EKF covariance propagation during relaxation.
% The helper preserves the original node.value and node.parent outputs.
[node, distance, predecessor] = dijkstra_ekf_prm(node, neighbor_ID, F, R, ...
    alpha, obstacle_edge, chi, bound, num_props, prop);

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

%% Output Data
%%%%%%%%%%%%%% All data should be saved here %%%%%%%%%%%%%%%
% The file name used for save the data
% Data is saved in "data" folder
% Name includes N, alpha value, safety percentage 
savename = ['data/PRM_stigmergy_N', num2str(N), '_alpha_', num2str(alpha), ...
    '_radius_', num2str(connection_radius)];
savename(savename=='.') = [];
save(savename)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeElapsed = toc;
