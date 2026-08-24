function path = reconstruct_prm_path(predecessor, target_ID, source_ID)
%RECONSTRUCT_PRM_PATH Returns the source-to-target vertex sequence.

% Start at the target and follow predecessor links back to the source.
path = zeros(numel(predecessor), 1);
path(1) = target_ID;
path_length = 1;
current_ID = target_ID;

while current_ID ~= source_ID
    current_ID = predecessor(current_ID);

    % A zero predecessor means the target is unreachable.
    if current_ID == 0
        path = [];
        return
    end

    path_length = path_length + 1;
    path(path_length) = current_ID;
end

% The predecessor walk is target-to-source; reverse it for plotting.
path = flipud(path(1:path_length));
end
