function lb = mst_lower_bound(wp_list, x0)
%MST_LOWER_BOUND  Cheap lower bound on the optimal waypoint path length.
%
%   Any path that starts at X0 and visits every waypoint is itself a spanning
%   tree of the point set, so the minimum spanning tree can never be longer
%   than the optimal path. That makes MST length a valid lower bound at any N,
%   computable in O(N^2) by Prim's algorithm, for use where exact solution is
%   out of reach.
%
%   The bound is loose - on uniform random points it typically sits 10-25%
%   below the true optimum - so a gap measured against it overstates every
%   algorithm's gap by roughly the same amount. Use it to compare algorithms
%   to each other at large N, not to claim an absolute optimality gap. Run it
%   alongside the exact solver in the range where both work and the offset
%   between the two curves tells you how loose the bound is on your instances.

P = [x0; wp_list];
n = size(P, 1);

D = hypot(P(:,1) - P(:,1).', P(:,2) - P(:,2).');

inT     = false(n, 1);
inT(1)  = true;
best    = D(:, 1);      % cheapest known edge from the tree to each vertex
best(1) = inf;
lb      = 0;

for i = 2:n
    [w, j]  = min(best);
    lb      = lb + w;
    inT(j)  = true;
    best(j) = inf;

    upd       = ~inT & D(:,j) < best;
    best(upd) = D(upd, j);
end

end