function [wp_ordered, dist_cum] = wp_brute_force(wp_list, x0)

    N = size(wp_list, 1);
    
    P = perms(1:N);              % every ordering, one per row
    
    tot = vecnorm(wp_list(P(:,1),:) - x0, 2, 2);
    
    for k = 1:N-1
        tot = tot + vecnorm(wp_list(P(:,k+1),:) - wp_list(P(:,k),:), 2, 2);
    end
    
    [dist_cum, best] = min(tot);
    
    wp_ordered = wp_list(P(best,:), :);

end