%% tune_aco.m - constrained Bayesian optimisation of wp_antColony
%
%   minimise    iterations * ants          (tour constructions = compute cost)
%   subject to  hit rate >= TARGET         (fraction of runs reaching the
%                                           provable brute-force optimum)
%
% The objective is deterministic and costs nothing to evaluate - it is just
% the product of two search variables. All the uncertainty is in the
% constraint, which is what bayesopt's coupled-constraint machinery is for.
%
% Requires: Statistics and Machine Learning Toolbox (bayesopt).

clear; clc;

%% ---- Settings -------------------------------------------------------
K        = 6;       % waypoint instances to tune against
N_wp     = 10;
coord_lo = -100;
coord_hi =  100;
x0       = [0 0];

TARGET   = 0.90;    % required fraction of runs hitting the optimum
REPS     = 8;       % runs per instance per evaluation -> K*REPS samples
MAX_EVAL = 120;     % bayesopt budget

%% ---- Instance set ---------------------------------------------------
% Fixed rng so repeated tuning runs see the same problems. Several
% instances so we tune the algorithm rather than one particular geometry.

rng(2026);

fprintf('Building %d instances and solving each exactly...\n', K);
instances = struct('wp', {}, 'L_opt', {});
for ii = 1:K
    wp = coord_lo + (coord_hi - coord_lo)*rand(N_wp, 2);
    [~, L_opt] = wp_brute_force(wp, x0);
    instances(ii).wp    = wp;
    instances(ii).L_opt = L_opt;
    fprintf('  instance %d: optimal tour = %.2f m\n', ii, L_opt);
end

%% ---- Baseline for comparison ----------------------------------------
base = struct('iterations',150, 'ants',20, 'evaporation_coef',0.15, ...
              'Q',20, 'alpha',1, 'beta',3, 'pheromone_max',10);

fprintf('\nBaseline (150 x 20)...\n');
[h0, g0, t0] = evaluate_config(instances, x0, base, 25);
fprintf('  hit rate %.1f%%   mean gap %.3f%%   %.3f s   cost %d\n', ...
        100*h0, 100*g0, t0, base.iterations*base.ants);

%% ---- Search space ---------------------------------------------------
vars = [ optimizableVariable('iterations',    [5  150], 'Type','integer')
         optimizableVariable('ants',          [3   40], 'Type','integer')
         optimizableVariable('alpha',         [0     3])
         optimizableVariable('beta',          [1     6])
         optimizableVariable('rho',           [0.02  0.6])
         optimizableVariable('Q',             [1   100])
         optimizableVariable('pheromone_max', [1    50]) ];

%% ---- Optimise -------------------------------------------------------
fun = @(t) aco_cost(t, instances, x0, REPS, TARGET);

results = bayesopt(fun, vars, ...
    'NumCoupledConstraints',              1,   ...
    'IsObjectiveDeterministic',           true, ...
    'AreCoupledConstraintsDeterministic', false, ...
    'AcquisitionFunctionName',            'expected-improvement-plus', ...
    'MaxObjectiveEvaluations',            MAX_EVAL, ...
    'Verbose',                            1);

best = bestPoint(results);
disp('Best feasible hyperparameters:');
disp(best);

%% ---- Validate the winner on fresh runs ------------------------------
% bestPoint is chosen using REPS samples per instance, which is thin.
% Re-measure with many more before trusting it.

opt_best = struct('iterations', best.iterations, 'ants', best.ants, ...
                  'alpha', best.alpha, 'beta', best.beta, ...
                  'evaporation_coef', best.rho, 'Q', best.Q, ...
                  'pheromone_max', best.pheromone_max);

fprintf('\nValidating tuned config on 25 reps per instance...\n');
[h1, g1, t1] = evaluate_config(instances, x0, opt_best, 25);

cost0 = base.iterations*base.ants;
cost1 = double(opt_best.iterations)*double(opt_best.ants);

fprintf('\n--- Baseline vs tuned ------------------------------------\n');
fprintf('           iterations   ants   cost   hit rate   mean gap    time\n');
fprintf('baseline   %9d  %5d  %5d   %6.1f%%   %7.3f%%  %.3f s\n', ...
        base.iterations, base.ants, cost0, 100*h0, 100*g0, t0);
fprintf('tuned      %9d  %5d  %5d   %6.1f%%   %7.3f%%  %.3f s\n', ...
        opt_best.iterations, opt_best.ants, cost1, 100*h1, 100*g1, t1);
fprintf('compute reduction: %.1fx\n', cost0/cost1);

%% =====================================================================
function [J, cons] = aco_cost(t, instances, x0, reps, target)
% Objective: tour constructions performed. Deterministic, free to evaluate.
% Constraint: cons <= 0 means feasible, i.e. hit rate met the target.

    opt = struct('iterations', t.iterations, 'ants', t.ants, ...
                 'alpha', t.alpha, 'beta', t.beta, ...
                 'evaporation_coef', t.rho, 'Q', t.Q, ...
                 'pheromone_max', t.pheromone_max);

    hits = false(numel(instances)*reps, 1);
    c = 0;
    for ii = 1:numel(instances)
        for r = 1:reps
            c = c + 1;
            [~, L] = wp_antColony(instances(ii).wp, x0, opt);
            hits(c) = (L - instances(ii).L_opt) / instances(ii).L_opt < 1e-4;
        end
    end

    J    = double(t.iterations) * double(t.ants);
    cons = target - mean(hits);
end

%% =====================================================================
function [hit_rate, gap_mean, t_mean] = evaluate_config(instances, x0, opt, reps)

    n    = numel(instances)*reps;
    hits = false(n,1);
    gaps = zeros(n,1);
    wall = zeros(n,1);

    c = 0;
    for ii = 1:numel(instances)
        for r = 1:reps
            c = c + 1;
            ts = tic;
            [~, L] = wp_antColony(instances(ii).wp, x0, opt);
            wall(c) = toc(ts);
            gaps(c) = (L - instances(ii).L_opt) / instances(ii).L_opt;
            hits(c) = gaps(c) < 1e-4;
        end
    end

    hit_rate = mean(hits);
    gap_mean = mean(gaps);
    t_mean   = mean(wall);
end