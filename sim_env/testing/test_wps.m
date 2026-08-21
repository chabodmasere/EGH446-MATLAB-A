%% TEST_WP_ORDER
% Runs n_trials independent random waypoint sets and orders each one three
% ways: nearest-neighbour heuristic, exhaustive search, and ant colony
% optimisation.
%
% All three algorithms see the SAME waypoint list within a trial, so the
% per-trial comparison is valid without fixing the random seed. Averaging
% over many trials separates real performance differences from the noise of
% a single lucky or unlucky instance.
%
% Brute force is the exact optimum, so it doubles as the reference against
% which the two heuristics are scored.

clear; close all; clc;


%% Configuration

n_trials = 1000;
N        = 10;          % perms(1:N) allocates N!-by-N doubles - see note below
x0       = [0, 0];

coord_lo = -100;
coord_hi =  100;


%% Preallocate

dist_nn    = zeros(n_trials,1);   time_nn    = zeros(n_trials,1);
dist_brute = zeros(n_trials,1);   time_brute = zeros(n_trials,1);
dist_aco   = zeros(n_trials,1);   time_aco   = zeros(n_trials,1);

% Kept so the worst ACO trial can be plotted afterwards for diagnosis.
wp_lists  = cell(n_trials,1);
tours_nn  = cell(n_trials,1);
tours_bf  = cell(n_trials,1);
tours_aco = cell(n_trials,1);


%% Run the trials

fprintf('Running %d trials at N = %d ...\n', n_trials, N);

for t = 1:n_trials

    X = randi([coord_lo, coord_hi], N, 1);
    Y = randi([coord_lo, coord_hi], N, 1);
    wp_list = [X Y];

    [wp_n, d_n, tm_n, wp_b, d_b, tm_b, wp_a, d_a, tm_a] = wp_order(wp_list);

    dist_nn(t)    = d_n;   time_nn(t)    = tm_n;
    dist_brute(t) = d_b;   time_brute(t) = tm_b;
    dist_aco(t)   = d_a;   time_aco(t)   = tm_a;

    wp_lists{t}  = wp_list;
    tours_nn{t}  = wp_n;
    tours_bf{t}  = wp_b;
    tours_aco{t} = wp_a;

    fprintf('  trial %2d/%d\n', t, n_trials);
end

fprintf('\n');


%% Per-trial optimality gap
% Raw distances are not comparable across trials because each instance has a
% different optimal length. The gap relative to that trial's own optimum is,
% so gaps are what get averaged.

gap_nn  = (dist_nn  - dist_brute) ./ dist_brute * 100;
gap_aco = (dist_aco - dist_brute) ./ dist_brute * 100;

tol      = 1e-9;
hit_nn   = mean(gap_nn  < tol) * 100;    % percent of trials that found the optimum
hit_aco  = mean(gap_aco < tol) * 100;


%% Report

fprintf('======================================================================\n');
fprintf(' %d trials, N = %d waypoints, %d permutations per exhaustive search\n', ...
        n_trials, N, factorial(N));
fprintf('======================================================================\n\n');

fprintf('%-20s %14s %14s %14s\n', '', 'Mean dist (m)', 'Mean time (s)', 'Std time (s)');
fprintf('%-20s %14.2f %14.4f %14.4f\n', 'Nearest neighbour', ...
        mean(dist_nn),    mean(time_nn),    std(time_nn));
fprintf('%-20s %14.2f %14.4f %14.4f\n', 'Brute force', ...
        mean(dist_brute), mean(time_brute), std(time_brute));
fprintf('%-20s %14.2f %14.4f %14.4f\n', 'Ant colony', ...
        mean(dist_aco),   mean(time_aco),   std(time_aco));

fprintf('\n');
fprintf('%-20s %10s %10s %10s %12s\n', ...
        'Gap vs optimum', 'Mean %', 'Worst %', 'Std %', 'Optimal %');
fprintf('%-20s %10.2f %10.2f %10.2f %12.0f\n', 'Nearest neighbour', ...
        mean(gap_nn),  max(gap_nn),  std(gap_nn),  hit_nn);
fprintf('%-20s %10.2f %10.2f %10.2f %12.0f\n', 'Ant colony', ...
        mean(gap_aco), max(gap_aco), std(gap_aco), hit_aco);

fprintf('\n');
fprintf('Brute force is %.0fx slower than NN and %.0fx slower than ACO.\n', ...
        mean(time_brute) / mean(time_nn), mean(time_brute) / mean(time_aco));
fprintf('ACO is %.0fx slower than NN for a %.2f%% shorter tour on average.\n', ...
        mean(time_aco) / mean(time_nn), mean(gap_nn) - mean(gap_aco));
fprintf('\n');


%% Figure 1 - worst ACO trial, plotted three ways
% The worst case is more diagnostic than a typical one: if ACO fails, this
% shows where in the tour it went wrong.

[~, t_worst] = max(gap_aco);

wp_list = wp_lists{t_worst};
all_pts = [wp_list; x0];

pad   = 0.1 * max(max(all_pts(:,1)) - min(all_pts(:,1)), ...
                  max(all_pts(:,2)) - min(all_pts(:,2)));
xlims = [min(all_pts(:,1)) - pad, max(all_pts(:,1)) + pad];
ylims = [min(all_pts(:,2)) - pad, max(all_pts(:,2)) + pad];

figure('Name', 'Worst ACO trial', 'Position', [100 100 1350 450]);

plot_tour(1, tours_nn{t_worst},  x0, dist_nn(t_worst),    time_nn(t_worst), ...
          'Nearest Neighbour', xlims, ylims);
plot_tour(2, tours_bf{t_worst},  x0, dist_brute(t_worst), time_brute(t_worst), ...
          'Brute Force',       xlims, ylims);
plot_tour(3, tours_aco{t_worst}, x0, dist_aco(t_worst),   time_aco(t_worst), ...
          'Ant Colony',        xlims, ylims);

sgtitle(sprintf('Worst ACO trial (#%d, gap %.2f%%)', t_worst, gap_aco(t_worst)));


%% Figure 2 - distribution of the optimality gap

figure('Name', 'Performance summary', 'Position', [100 600 1000 400]);

subplot(1,2,1);
plot(1:n_trials, gap_nn,  'o-', 'LineWidth', 1.1, 'DisplayName', 'Nearest neighbour');
hold on;
plot(1:n_trials, gap_aco, 's-', 'LineWidth', 1.1, 'DisplayName', 'Ant colony');
yline(0, 'k--', 'Optimum', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('Trial');
ylabel('Gap above optimum (%)');
title('Per-trial optimality gap');
legend('Location', 'best');
grid on;

subplot(1,2,2);
bar([mean(time_nn), mean(time_aco), mean(time_brute)]);
set(gca, 'YScale', 'log', 'XTickLabel', {'NN', 'ACO', 'Brute'});
ylabel('Mean runtime (s, log scale)');
title('Mean runtime');
grid on;


%% ------------------------------------------------------------------------
function plot_tour(pos, wp, x0, dist, t, name, xlims, ylims)

    subplot(1, 3, pos);

    % Full tour including the leg out from the start position.
    tour = [x0; wp];

    plot(tour(:,1), tour(:,2), '-o', 'LineWidth', 1.2, 'MarkerSize', 5);
    hold on;

    plot(x0(1), x0(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

    for i = 1:size(wp,1)
        text(wp(i,1), wp(i,2), sprintf('  %d', i), 'FontSize', 9);
    end

    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('%s\nDistance: %.2f m   Time: %.4f s', name, dist, t));

    grid on;
    axis equal;
    xlim(xlims);
    ylim(ylims);

end