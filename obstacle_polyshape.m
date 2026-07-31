function obs_poly = obstacle_polyshape()
%This function defines an obstacle as a set of polyshapes
% This function is defined to use polyshape functionalities of Matlab

n=2; % number of obstacles
obs_poly(1:n)= struct('x',[], 'y',[]);

% Obstacle_1
obs_poly(1).x = [0.4 0.5 0.5 0.4];
obs_poly(1).y = [0.0 0.0 0.45 0.45];

% Obstacle_2
obs_poly(2).x = [0.4 0.4 0.5 0.4];
obs_poly(2).y = [0.65 0.65 1.0 1.0];