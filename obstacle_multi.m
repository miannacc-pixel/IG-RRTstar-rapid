function obstacle_edge = obstacle_multi()
% Defines an obstacle as a set of edges 
% each edge is defined by: start point, end point, slope, and Y_axis
% intercept

% Obstacle_1
x(1).vertex = [0.4,0.0];
vertex_index= length(x);
x(end+1).vertex = [0.5,0.0];
x(end+1).vertex = [0.5,0.45]; 
x(end+1).vertex = [0.4,0.45]; 
for i=vertex_index:length(x)
obstacle_edge(i).start = x(i).vertex;
if (i~=length(x))
obstacle_edge(i).end = x(i+1).vertex;
else
   obstacle_edge(i).end = x(vertex_index).vertex; 
end
obstacle_edge(i).slope = (obstacle_edge(i).start(2)-obstacle_edge(i).end(2))/...
    (obstacle_edge(i).start(1)-obstacle_edge(i).end(1));
obstacle_edge(i).y_inter = obstacle_edge(i).start(2) -...
    obstacle_edge(i).slope*obstacle_edge(i).start(1);
end

% Obstacle_2
x(end+1).vertex = [0.4,0.65]; 
vertex_index= length(x);
x(end+1).vertex = [0.5,0.65];
x(end+1).vertex = [0.5,1.0];
x(end+1).vertex = [0.4,1.0];
for i=vertex_index:length(x)
obstacle_edge(i).start = x(i).vertex;
if (i~=length(x))
obstacle_edge(i).end = x(i+1).vertex;
else
   obstacle_edge(i).end = x(vertex_index).vertex; 
end
obstacle_edge(i).slope = (obstacle_edge(i).start(2)-obstacle_edge(i).end(2))/...
    (obstacle_edge(i).start(1)-obstacle_edge(i).end(1));
obstacle_edge(i).y_inter = obstacle_edge(i).start(2) -...
    obstacle_edge(i).slope*obstacle_edge(i).start(1);
end