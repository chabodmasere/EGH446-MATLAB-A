function [wp_ordered, dist, order] = tsp_exact_hk(wp_list, x0)
%TSP_EXACT_HK  Exact shortest open path from x0 through every waypoint.
%
%   Returns the same answer as exhaustive search, but by dynamic programming
%   (Held-Karp) rather than enumeration: O(2^N * N^2) work and O(2^N * N)
%   memory instead of O(N!).
%
%       N      perms(1:N) rows        DP states (2^N * N)
%       10          3,628,800                    10,240
%       11         39,916,800                    22,528
%       12        479,001,600                    49,152
%       15      1.31 x 10^12                    491,520
%       18      6.40 x 10^15                  4,718,592
%
%   perms(1:N) allocates an N!-by-N double array, so N = 11 already asks for
%   3.5 GB and N = 12 asks for 42 GB. That, not the ant colony, is what stops
%   the existing test harness from going past N = 10.
%
%   Practical ceiling here is about N = 18 (roughly 40 MB of table, a few
%   seconds per instance). N = 20 needs ~170 MB and a minute or two. Above
%   that, use a bound instead - see mst_lower_bound.
%
%   [WP_ORDERED, DIST, ORDER] = TSP_EXACT_HK(WP_LIST, X0)
%       WP_LIST     N-by-2 waypoint coordinates
%       X0          1-by-2 start position (departed from, not returned to)
%       WP_ORDERED  N-by-2, WP_LIST rearranged into the optimal visit order
%       DIST        total path length, including the leg out from X0
%       ORDER       1-by-N index vector, so WP_ORDERED = WP_LIST(ORDER,:)
%
%   Note this solves an OPEN path, matching the existing distance and plot
%   convention ([x0; wp] with no return leg). It is not the closed-tour TSP.

N = size(wp_list, 1);

if N > 22
    error('tsp_exact_hk:tooLarge', ...
          'N = %d would need %.1f GB for the DP table.', N, 2^N * N * 8 / 1e9);
end

% Point 1 is the start; points 2..N+1 are the waypoints.
P  = [x0; wp_list];
Dp = hypot(P(:,1) - P(:,1).', P(:,2) - P(:,2).');
d0 = Dp(1, 2:end);              % start -> waypoint j
D  = Dp(2:end, 2:end);          % waypoint j -> waypoint k

nS = 2^N;

% dp(S+1, j) = length of the shortest path that leaves x0, visits exactly the
% waypoints in bit-set S, and finishes at j. Subsets are 0-based, MATLAB
% indices are not, hence the +1 everywhere.
dp     = inf(nS, N);
parent = zeros(nS, N, 'uint8');     % 0 = no predecessor, i.e. first waypoint

% Base case: one waypoint, straight out from the start.
for j = 1:N
    dp(bitshift(1, j-1) + 1, j) = d0(j);
end

% Extend every reachable state by one unvisited waypoint.
for S = 1:nS-1

    members = bitget(S, 1:N) == 1;

    js = find(members);
    ks = find(~members);
    if isempty(ks)
        continue                    % full set, nothing left to append
    end

    costs = dp(S+1, js);
    live  = isfinite(costs);
    if ~any(live)
        continue                    % state not reachable
    end
    js    = js(live);
    costs = costs(live);

    % |js|-by-|ks|: cost of reaching each j, then stepping out to each k.
    [best, via] = min(costs(:) + D(js, ks), [], 1);

    for a = 1:numel(ks)
        k  = ks(a);
        S2 = S + bitshift(1, k-1);
        if best(a) < dp(S2+1, k)
            dp(S2+1, k)     = best(a);
            parent(S2+1, k) = js(via(a));
        end
    end
end

% Best endpoint once every waypoint has been visited.
S = nS - 1;
[dist, j] = min(dp(S+1, :));

% Walk the parent pointers back to the start.
order = zeros(1, N);
for i = N:-1:1
    order(i) = j;
    pj = parent(S+1, j);
    S  = S - bitshift(1, j-1);
    j  = double(pj);
end

wp_ordered = wp_list(order, :);

end