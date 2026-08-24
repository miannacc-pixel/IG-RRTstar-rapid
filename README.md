# IG-RRTstar-rapid
===============================================================================================================================
Basic information
First, create a empty folder "data", where the data will be saved.
By running "main.m", you can start the information-geometric PRM path-planning algorithm.
===============================================================================================================================

Codes related with the Path-planning simulation:

main.m: The main function to run the directed information-geometric PRM

dijkstra_prm.m: Computes shortest paths on the completed directed roadmap

reconstruct_prm_path.m: Reconstructs the final source-to-target PRM path

obstacle_multi.m & obstacle_polyshape.m :  specify the obstacles position and their shape for a sample "multiple obstacles" enviroment

sample_x_P_randomly.m: It samples x and P randomly 

sample_polyshape_check.m: samples x outside of all obstacle. It is used in sample_x_P_randomly.m

sample_x_P_randomly.m: It samples collision-free belief states for roadmap vertices

error_ellipse.m: Calculate the length of the long & short axes, angle, and bounding box of a given ellipse

dist_ig_mat.m & dist_ig_mat2.m: The functions which calculate the RI-distance. The difference between these two functions is whether we want to measure the distance from the set of nodes to one node, or from one node to the set of nodes.

Q_hat_sol.m: It computes information cost using SVD

check_lossless.m: Checks whether a directed roadmap transition is lossless


******** The function related for obstacle checking ********
boundary_check.m: It checks whether an ellipse intersect with regions's boundary

psuedo_obs_check_line_oct.m: It checks if a collision with obstacles during the transition between nodes occurs. 
psuedo_obs_check_line2_oct.m: The same with the above 

Is_two_lineseg_cross.m: check if two line segment intersect
make_octagon.m: generate a octagon from a bounding box for better obstacle checking
minDist_two_LineSeg_in.m: compute the minimum distance between two line segments. It is used for collision pre-checking
**********************************************************
===============================================================================================================================
Codes related with the plotting
plot_for_paper_multipleObs: This function plots results for "multiple obstacles" enviroment
plot_path_cost: It plots the path length as a function of number of samples. It should be run right after main.m
===============================================================================================================================
