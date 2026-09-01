function [in_range, fraction] = marker_encounter(x_start, x_end, marker)
%MARKER_ENCOUNTER Tests whether a directed edge enters marker sensing range.
edge=x_end-x_start; length_squared=dot(edge,edge);
if length_squared==0, fraction=0; else, fraction=dot(marker.x-x_start,edge)/length_squared; fraction=min(max(fraction,0),1); end
in_range=norm(x_start+fraction*edge-marker.x)<=marker.sensing_radius;
end
