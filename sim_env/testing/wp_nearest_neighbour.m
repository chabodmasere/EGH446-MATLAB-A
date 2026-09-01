function [wp_ordered, dist_cum] = wp_nearest_neighbour(wp_list, x0)
%WP_NEAREST_NEIGHBOUR  Greedy nearest-neighbour waypoint ordering.
%
%   Pulled out of wp_order.m unchanged so it can be timed on its own without
%   dragging brute force along. Delete the local copy in wp_order.m - a local
%   function shadows a file of the same name inside its own file, so leaving
%   both in place means two copies that will drift apart.

N          = size(wp_list, 1);
wp_ordered = zeros(N, 2);
unvisited  = true(N, 1);
xc         = x0;
dist_cum   = 0;

for j = 1:N
    d            = inf(N, 1);
    d(unvisited) = vecnorm(wp_list(unvisited,:) - xc, 2, 2); % checks distance of all remaining

    [dist_min, idx] = min(d);

    dist_cum        = dist_cum + dist_min;
    xc              = wp_list(idx,:);
    wp_ordered(j,:) = xc;
    unvisited(idx)  = false;
end

end