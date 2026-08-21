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

    % variable initialisation
    wp_ordered = zeros(N, 2);
    dist_cum   = 0;
    probabilities = zeros(N,1);

    path_pheramones = starting_pheramones* ones(N+1);   % matrix storage of "pheromone" levels

    for i = 1:iterations % once finished highest pheromones is path 
        ants_path = zeros(N*ants,2);
        for j = 1:ants % run set of 5 ants to add pheromones
            unvisted_wps = true(N,1);
            xc = x0;
            ant_dist = 0;
            
            for k = 1:N % iterate through path
                probabilities = zeros(N,1);
                for m = 1:N % iterate through waypoints to work out where to go
                    % work out N's in probability formula
                    if unvisted_wps(m) == 0
                        continue
                    end
                    d = zeros(N,1);
                    d(unvisted_wps) = vecnorm(wp_list(unvisted_wps,:)- xc, 2, 2); % works out distance to each unvisited wp
                    d_k = vecnorm(wp_list(m,:) - xc, 2, 2); % proximity to specific wp Nij

                    % work out tau's in probability formula
                    mask = false(size(path_pheramones));

                    if xc == x0
                        mask(1,:) = true;
                        mask(:,1) = true;
                        index_xc = 1;
                        mask(index_xc,index_xc) = false; % remove itself
                    else 
                        index_xc = find(wp_list(:,1) == xc(1) & wp_list(:,2) == xc(2))+1;
                        mask(index_xc,2:end) = true; % starts at 2 so x0 isn't included
                        mask(2:end,index_xc) = true;
                        mask(index_xc,index_xc) = false; % remove itself
                        % make visited wp's false
                        index_visted = find(unvisted_wps == false);
                        for l = index_visted
                            mask(:,l+1) = false;
                        end
                    end
                    pheromones_masked = mask .* path_pheramones; 
                    path_pheromones_m = pheromones_masked(index_xc,m+1); % pheromones on specific path
                    probabilities(m) = path_pheromones_m^alpha * (1/d_k)^beta;
                end
                weight_total = sum(probabilities);
                probabilities = probabilities / weight_total;

                dec = rand();
                idx = find(cumsum(probabilities) > dec, 1);
                ant_dist = ant_dist + norm(wp_list(idx,:) - xc);  
                xc = wp_list(idx, :);
                ants_path((j-1)*N+k,:) = xc;
                unvisted_wps(idx) = false; 
            end
            if ant_dist < best_dist
                best_dist = ant_dist;
                best_path = ants_path((j-1)*N+1 : j*N, :);
            end
        end

        path_pheramones = pheromone_retention * path_pheramones; % scaled (1-p)*Tij(t)
        for j = 1:length(ants_path)
            if mod(j,N) == 1
               index_c = 1;
            else
                x_c = ants_path(j-1,:);
               index_c = find(wp_list(:,1) == x_c(1) & wp_list(:,2) == x_c(2)) + 1;
            end
            x_n = ants_path(j,:);
            index_n = find(wp_list(:,1) == x_n(1) & wp_list(:,2) == x_n(2)) + 1;
            ant_no = ceil((j)/N);
            ant_path = ants_path((ant_no-1)*N+1:ant_no*N,:);
            ant_path = [x0; ant_path];              
            seg_vectors = diff(ant_path, 1, 1);      % (N-1) x dims, consecutive waypoint deltas
            seg_lengths = vecnorm(seg_vectors, 2, 2); % Euclidean norm of each row -> (N-1) x 1
            total_distance = sum(seg_lengths);
            deposit = Q/total_distance;
            path_pheramones(index_c,index_n) = path_pheramones(index_c,index_n) + deposit;
            path_pheramones(index_n,index_c) = path_pheramones(index_n,index_c) + deposit;

        end
        path_pheramones = min(max(path_pheramones, pheromone_min), pheromone_max);

    end
    wp_ordered = best_path;
    dist_cum   = best_dist;
end