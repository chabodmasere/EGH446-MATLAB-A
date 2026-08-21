function [wp_ordered, dist_cum] = wp_antColony(wp_list, x0)
    % constants
    N = size(wp_list, 1);

    % tuneability
    iterations = 150;
    ants = 20;
    evaporation_coef = 0.15;
    pheromone_retention = 1 - evaporation_coef;
    Q = 20;
    alpha = 1;
    beta = 3;
    pheromone_max = 10;
    pheromone_min = pheromone_max/(2*N);
    starting_pheramones = pheromone_max;


    best_dist = inf;
    best_path = zeros(N,2);

    path_pheramones = starting_pheramones* ones(N+1);   % matrix storage of "pheromone" levels

    for i = 1:iterations % once finished highest pheromones is path
        ants_path = zeros(N*ants,2);
        ants_idx  = zeros(N*ants,1);   % waypoint index visited at each step
        ant_dists = zeros(ants,1);     % tour length of each ant this iteration

        for j = 1:ants % run set of ants to add pheromones
            unvisted_wps = true(N,1);
            xc = x0;
            ant_dist = 0;

            for k = 1:N % iterate through path
                % index of the current node in path_pheramones.
                % row/col 1 is the depot x0, row/col w+1 is waypoint w.
                % at k == 1 the ant is still at x0; after that the ant is
                % sitting on the waypoint it chose last step, so idx carries
                % over and no lookup into wp_list is needed.
                if k == 1
                    index_xc = 1;
                else
                    index_xc = idx + 1;
                end

                probabilities = zeros(N,1);
                for m = 1:N % iterate through waypoints to work out where to go
                    if ~unvisted_wps(m)
                        continue
                    end
                    d_k = norm(wp_list(m,:) - xc);              % Nij, proximity to wp m
                    tau = path_pheramones(index_xc, m+1);       % Tij, trail on this edge
                    probabilities(m) = tau^alpha * (1/d_k)^beta;
                end

                weight_total  = sum(probabilities);
                probabilities = probabilities / weight_total;

                dec = rand();
                idx = find(cumsum(probabilities) > dec, 1);
                if isempty(idx)
                    % cumsum can land a hair under 1 through float error, and
                    % with ~1e5 draws per call that does eventually happen.
                    idx = find(unvisted_wps, 1, 'last');
                end

                ant_dist = ant_dist + norm(wp_list(idx,:) - xc);
                xc = wp_list(idx, :);
                ants_path((j-1)*N+k,:) = xc;
                ants_idx((j-1)*N+k)    = idx;
                unvisted_wps(idx)      = false;
            end

            ant_dists(j) = ant_dist;

            if ant_dist < best_dist
                best_dist = ant_dist;
                best_path = ants_path((j-1)*N+1 : j*N, :);
            end
        end

        path_pheramones = pheromone_retention * path_pheramones; % scaled (1-p)*Tij(t)

        for j = 1:N*ants
            if mod(j,N) == 1
                index_c = 1;                 % leg out of the depot
            else
                index_c = ants_idx(j-1) + 1;
            end
            index_n = ants_idx(j) + 1;

            ant_no  = ceil(j/N);
            deposit = Q / ant_dists(ant_no); % tour length already accumulated above

            path_pheramones(index_c,index_n) = path_pheramones(index_c,index_n) + deposit;
            path_pheramones(index_n,index_c) = path_pheramones(index_n,index_c) + deposit;
        end

        path_pheramones = min(max(path_pheramones, pheromone_min), pheromone_max);

    end

    wp_ordered = best_path;
    dist_cum   = best_dist;
end