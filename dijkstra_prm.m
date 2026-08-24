function [distance, predecessor] = dijkstra_prm(edge_cost, source_ID)
%DIJKSTRA_PRM Finds shortest paths in a directed PRM roadmap.
% edge_cost(i,j) is the nonnegative cost of the directed edge i -> j.
% An absent edge is represented by Inf.

num_nodes = size(edge_cost, 1);
distance = inf(num_nodes, 1);
predecessor = zeros(num_nodes, 1);
visited = false(num_nodes, 1);
distance(source_ID) = 0;

% At each step, settle the unsettled vertex with the lowest known cost.
for ii = 1:num_nodes
    unsettled_distance = distance;
    unsettled_distance(visited) = inf;
    [current_distance, current_ID] = min(unsettled_distance);

    % Remaining vertices are unreachable from the source vertex.
    if isinf(current_distance)
        break
    end

    visited(current_ID) = true;
    neighbor_ID = find(isfinite(edge_cost(current_ID,:)));

    % Relax every outgoing directed edge from the current roadmap vertex.
    for jj = 1:numel(neighbor_ID)
        next_ID = neighbor_ID(jj);
        candidate_distance = current_distance + edge_cost(current_ID,next_ID);

        if candidate_distance < distance(next_ID)
            distance(next_ID) = candidate_distance;
            predecessor(next_ID) = current_ID;
        end
    end
end
end
