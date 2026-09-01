function edge_cost = dist_stigmergy_mat(Dtravel, P_current, P_next, lambda)
%DIST_STIGMERGY_MAT Computes one edge increment for the stigmergy objective.
%
% With the source initialized to lambda*trace(P_initial), these increments
% telescope to Dtotal = sum(Dtravel) + lambda*trace(P_goal).

edge_cost = Dtravel + lambda * (trace(P_next) - trace(P_current));
end