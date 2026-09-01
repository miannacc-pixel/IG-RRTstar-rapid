function x = sample_free_position(bound, obs_polyshape)
%SAMPLE_FREE_POSITION Samples a spatial PRM vertex outside all obstacles.

while true
    x = [bound(1).x(1) + diff(bound(1).x) * rand, ...
         bound(2).x(1) + diff(bound(2).x) * rand];
    inside_obstacle = false;

    for kk = 1:numel(obs_polyshape)
        if inpolygon(x(1), x(2), obs_polyshape(kk).x, obs_polyshape(kk).y)
            inside_obstacle = true;
            break
        end
    end

    if ~inside_obstacle
        return
    end
end
end
