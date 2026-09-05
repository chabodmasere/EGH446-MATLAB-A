function stats = plot_lkf_results(src, opts)
% PLOT_LKF_RESULTS  Console metrics and diagnostic figures for the estimator.
%
% Usage
%   plot_lkf_results               metrics + all figures (saved as PNG)
%   plot_lkf_results(Plots=false)  metrics only, no figure windows
%   stats = plot_lkf_results(out)  read logs from a sim output object
%
% Primary use: the 7-state linear KF ([x y th v w vx vy]).
% Also runs on the 5-state EKF comparison case ([x y th v w]).
% State labels adapt automatically.
%
% Consistency metrics are computed on the SAME steady-state window.

arguments
    src = []
    opts.dt      (1,1) double  = 0.01
    opts.k0      (1,1) double  = 500
    opts.Plots   (1,1) logical = true
    opts.Save    (1,1) logical = true
    opts.SaveDir (1,:) char    = fullfile(fileparts(mfilename('fullpath')), 'figures')
    opts.Prefix  (1,:) char    = 'partB'
end

dt = opts.dt;
tag = 'lkf_partB';

set(0, 'DefaultFigureColor', 'w');

try
    set(groot, 'defaultFigureTheme', 'light');
catch
end

% Close only figures from a previous run of this function.
close(findobj('Type', 'figure', 'Tag', tag));

%% ---------------- Load logged signals ----------------
names = {'pose_log', 'z_log', 'xhat_log', ...
         'K_log', 'P_log', 'innov_log'};

S = load_signals(src, names);

[xhat,  t_x] = grab(S.xhat_log,  dt);
[pose,  t_p] = grab(S.pose_log,  dt);
[z,     t_z] = grab(S.z_log,     dt);
[innov, t_i] = grab(S.innov_log, dt);
[K,     t_K] = grab3(S.K_log,   dt);
[P,     t_P] = grab3(S.P_log,   dt);

t = t_x(:);

pose  = align(pose,  t_p, t);
z     = align(z,     t_z, t);
innov = align(innov, t_i, t);
K     = align3(K, t_K, t);
P     = align3(P, t_P, t);

N = numel(t);
k = (1:N)';
ns = size(P, 1);

state_names = {'x', 'y', '\theta', 'v', '\omega', 'v_x', 'v_y'};
state_names = state_names(1:min(ns, numel(state_names)));

if abs(numel(t_p) - N) > 1
    fprintf([ ...
        'Note: pose_log had %d samples but xhat_log had %d. They were ', ...
        'resampled onto a common grid. Set every To Workspace block to ', ...
        'Sample time 0.01 to avoid this.\n\n'], numel(t_p), N);
end

wrap = @(a) atan2(sin(a), cos(a));

%% ---------------- Errors ----------------
err = pose - xhat;
err(:,3) = wrap(err(:,3));

meas_err = pose - z(:,1:3);
meas_err(:,3) = wrap(meas_err(:,3));

err_sq = err.^2;

% One window for every consistency-relevant quantity.
k0 = min(opts.k0, N);
ss = k0:N;

% Steady-state RMSE.
rmse_est  = sqrt(mean(err(ss,:).^2));
rmse_meas = sqrt(mean(meas_err(ss,:).^2));

% Full-run RMSE, reported separately.
rmse_est_full  = sqrt(mean(err.^2));
rmse_meas_full = sqrt(mean(meas_err.^2));

% Filter uncertainty over the same window.
Pd = zeros(N, ns);

for i = 1:ns
    Pd(:,i) = squeeze(P(i,i,:));
end

sigma_P = sqrt(mean(Pd(ss,1:3)));
consist = rmse_est ./ sigma_P;

% Innovation variance over the same steady-state window.
innov_var = var(innov(ss,:));

fprintf('Steady-state window: k = %d..%d of %d samples\n\n', k0, N, N);
fprintf('              x         y        theta\n');
fprintf('RMSE raw    %8.4f  %8.4f  %8.4f\n', rmse_meas);
fprintf('RMSE est    %8.4f  %8.4f  %8.4f\n', rmse_est);
fprintf('Improvement %8.2fx %8.2fx %8.2fx\n', rmse_meas ./ rmse_est);

fprintf( ...
    'sqrt(P_ii)  %8.4f  %8.4f  %8.4f   (filter''s own 1-sigma)\n', ...
    sigma_P);

fprintf([ ...
    'RMSE/sqrtP  %8.2f  %8.2f  %8.2f   ', ...
    '(consistency: ~1 honest, >1.5 over-confident)\n'], consist);

fprintf([ ...
    '(full-run RMSE incl. transient: est %.4f %.4f %.4f | ', ...
    'raw %.4f %.4f %.4f)\n'], rmse_est_full, rmse_meas_full);

fprintf('\nSteady-state covariance P (last step):\n');
disp(P(:,:,end));

fprintf('Steady-state Kalman gain K (last step):\n');
disp(K(:,:,end));

fprintf('Innovation variance over window (should be near R + H P H''):\n');
disp(innov_var);

stats = struct( ...
    't', t, ...
    'err', err, ...
    'meas_err', meas_err, ...
    'k0', k0, ...
    'rmse_est', rmse_est, ...
    'rmse_meas', rmse_meas, ...
    'rmse_est_full', rmse_est_full, ...
    'rmse_meas_full', rmse_meas_full, ...
    'sigma_P', sigma_P, ...
    'consistency', consist, ...
    'P_ss', P(:,:,end), ...
    'K_ss', K(:,:,end), ...
    'innov_var', innov_var);

%% ---------------- Figures ----------------
if opts.Plots

    % Bold styling applies to every figure and subplot.
    newfig = @(name) figure( ...
        'Name', name, ...
        'Tag', tag, ...
        'Color', 'w', ...
        'Units', 'pixels', ...
        'Position', [100 80 1200 850], ...
        'DefaultAxesColor', 'w', ...
        'DefaultAxesXColor', 'k', ...
        'DefaultAxesYColor', 'k', ...
        'DefaultAxesFontName', 'Arial', ...
        'DefaultAxesFontSize', 14, ...
        'DefaultAxesFontWeight', 'bold', ...
        'DefaultAxesLineWidth', 2.0, ...
        'DefaultAxesTickLabelInterpreter', 'tex', ...
        'DefaultAxesTitleFontWeight', 'bold', ...
        'DefaultAxesTitleFontSizeMultiplier', 1.15, ...
        'DefaultAxesLabelFontSizeMultiplier', 1.10, ...
        'DefaultTextFontWeight', 'bold', ...
        'DefaultTextColor', 'k', ...
        'DefaultTextInterpreter', 'tex', ...
        'DefaultLegendFontSize', 13, ...
        'DefaultLegendFontWeight', 'bold', ...
        'DefaultLegendInterpreter', 'tex', ...
        'DefaultLegendColor', 'w', ...
        'DefaultLegendTextColor', 'k', ...
        'DefaultLineLineWidth', 3.0);

    % Unicode x-hat keeps mathematical notation with bold TeX text.
    x_hat_symbol = ['x' char(770)];

    %% ---------------- States against k ----------------
    labels = {'x  [m]', 'y  [m]', '\theta  [rad]'};

    newfig('State estimates vs k');

    pose_p = pose;
    z_p = z;

    pose_p(:,3) = wrap(pose_p(:,3));
    z_p(:,3) = wrap(z_p(:,3));

    for i = 1:3

        subplot(3,1,i);

        plot(k, z_p(:,i), '.', ...
            'Color', [0.55 0.55 0.55], ...
            'MarkerSize', 8);
        hold on;

        plot(k, pose_p(:,i), 'r-', ...
            'LineWidth', 3.2);

        plot(k, xhat(:,i), 'b-', ...
            'LineWidth', 3.0);

        ylabel(labels{i});
        grid on;

        if i == 1
            title('Measured data, true state (red) and estimate (blue)');
        end

        if i == 3
            ylabel('\theta  [rad, wrapped]');
            xlabel('k');
        end

    end

    legend('measured', 'true', 'estimate', ...
        'Location', 'best');

    %% ---------------- x-y trajectory ----------------
    newfig('x-y trajectory');

    plot(z(:,1), z(:,2), '.', ...
        'Color', [0.55 0.55 0.55], ...
        'MarkerSize', 8);
    hold on;

    plot(pose(:,1), pose(:,2), 'r-', ...
        'LineWidth', 3.4);

    plot(xhat(:,1), xhat(:,2), 'b-', ...
        'LineWidth', 3.0);

    axis equal;
    grid on;

    xlabel('x  [m]');
    ylabel('y  [m]');

    title('Vehicle path: measured (grey), true (red), estimated (blue)');

    legend('measured', 'true', 'estimate', ...
        'Location', 'best');

    %% ---------------- Error and squared error ----------------
    newfig('Estimation error');

    subplot(2,1,1);

    plot( ...
        k, err(:,1), 'b-', ...
        k, err(:,2), 'c-', ...
        k, err(:,3), 'm-', ...
        'LineWidth', 3.0);

    grid on;
    ylabel('error');

    title( ...
        ['Estimation error  x_k - ' x_hat_symbol '_k'], ...
        'Interpreter', 'tex');

    legend('x', 'y', '\theta', ...
        'Location', 'best');

    subplot(2,1,2);

    semilogy( ...
        k, err_sq(:,1), 'b-', ...
        k, err_sq(:,2), 'c-', ...
        k, err_sq(:,3), 'm-', ...
        'LineWidth', 3.0);

    grid on;

    xlabel('k');
    ylabel('error^2');
    title('Estimate error squared');

    ylim([1e-5 1]);

    %% ---------------- Estimator vs raw sensor ----------------
    newfig('Estimator vs raw sensor');

    subplot(2,1,1);

    plot(k, meas_err(:,1), '-', ...
        'Color', [0.60 0.60 0.60], ...
        'LineWidth', 2.0);
    hold on;

    plot(k, err(:,1), 'b-', ...
        'LineWidth', 3.0);

    grid on;
    ylabel('x error [m]');

    title('Raw measurement error (grey) vs estimator error (blue)');

    subplot(2,1,2);

    plot(k, meas_err(:,3), '-', ...
        'Color', [0.60 0.60 0.60], ...
        'LineWidth', 2.0);
    hold on;

    plot(k, err(:,3), 'b-', ...
        'LineWidth', 3.0);

    grid on;

    xlabel('k');
    ylabel('\theta error [rad]');

    %% ---------------- Consistency envelope ----------------
    newfig('Consistency envelope');

    enames = {'x error [m]', 'y error [m]', '\theta error [rad]'};

    for i = 1:3

        subplot(3,1,i);

        b = 2*sqrt(Pd(:,i));

        fill( ...
            [k; flipud(k)], ...
            [b; -flipud(b)], ...
            [0.85 0.9 1], ...
            'EdgeColor', 'none');
        hold on;

        plot(k, err(:,i), 'b-', ...
            'LineWidth', 3.0);

        grid on;

        inside = 100*mean(abs(err(ss,i)) <= b(ss));

        ylabel(enames{i});

        title(sprintf([ ...
            '%s: %.1f%% of samples inside \\pm2\\sigma  ', ...
            '(target 95%%),  RMSE/\\sigma_P = %.2f'], ...
            strtok(enames{i}), inside, consist(i)));

        if i == 3
            xlabel('k');
        end

    end

    legend('\pm 2\sigma from P', 'error', ...
        'Location', 'best');

    %% ---------------- Kalman gain ----------------
    kz = min(300, N);

    newfig('Kalman gain');

    subplot(3,1,1);

    plot( ...
        k, squeeze(K(1,1,:)), 'b-', ...
        k, squeeze(K(2,2,:)), 'c-', ...
        k, squeeze(K(3,3,:)), 'm-', ...
        k, squeeze(K(4,4,:)), 'g-', ...
        'LineWidth', 3.0);

    grid on;
    ylabel('gain');

    title([ ...
        'Diagonal Kalman gains, full run ', ...
        '(K_{x,x} lies under K_{y,y}: symmetric model)']);

    legend( ...
        'K_{x,x}', 'K_{y,y}', 'K_{\theta,\theta}', 'K_{v,v}', ...
        'Location', 'best');

    subplot(3,1,2);

    plot( ...
        k(1:kz), squeeze(K(1,1,1:kz)), 'b-', ...
        k(1:kz), squeeze(K(2,2,1:kz)), 'c-', ...
        k(1:kz), squeeze(K(3,3,1:kz)), 'm-', ...
        k(1:kz), squeeze(K(4,4,1:kz)), 'g-', ...
        'LineWidth', 3.0);

    grid on;
    ylabel('gain');

    title(sprintf( ...
        'Same gains, first %d steps: large while P is large, then settle', ...
        kz));

    subplot(3,1,3);

    if ns >= 7

        % Linear KF: heading -> omega, position -> Cartesian velocity.
        plot( ...
            k(1:kz), squeeze(K(5,3,1:kz)), 'm-', ...
            k(1:kz), squeeze(K(6,1,1:kz)), 'b-', ...
            'LineWidth', 3.0);

        grid on;

        legend('K_{\omega,\theta}', 'K_{v_x,x}', ...
            'Location', 'best');

        title([ ...
            'Cross gains, first steps: \theta meas \rightarrow \omega,  ', ...
            'x meas \rightarrow v_x']);

    else

        % EKF: heading -> omega and heading -> x.
        plot( ...
            k(1:kz), squeeze(K(5,3,1:kz)), 'm-', ...
            k(1:kz), squeeze(K(1,3,1:kz)), 'b-', ...
            'LineWidth', 3.0);

        grid on;

        legend('K_{\omega,\theta}', 'K_{x,\theta}', ...
            'Location', 'best');

        title( ...
            'Cross gains, first steps: heading measurement \rightarrow \omega and x');

    end

    xlabel('k');
    ylabel('gain');

    %% ---------------- Error covariance ----------------
    newfig('Error covariance');

    subplot(2,1,1);

    semilogy(k, Pd, ...
        'LineWidth', 3.0);

    grid on;
    ylabel('P_{ii}');

    title( ...
        'Diagonal of P, full run (P_{xx} lies under P_{yy}: symmetric model)');

    legend(state_names, ...
        'Location', 'best');

    subplot(2,1,2);

    semilogy(k(1:kz), Pd(1:kz,:), ...
        'LineWidth', 3.0);

    grid on;

    xlabel('k');
    ylabel('P_{ii}');

    title(sprintf([ ...
        'First %d steps: uncertainty collapses from the initial ', ...
        'guess to steady state'], kz));

    %% ---------------- Steady-state covariance ----------------
    newfig('Steady-state covariance');

    Pf = P(:,:,end);
    d = sqrt(diag(Pf));

    % Correlation matrix.
    C = Pf ./ (d*d');

    imagesc(C);
    caxis([-1 1]);
    axis square;

    cb = colorbar;

    set(cb, ...
        'FontName', 'Arial', ...
        'FontSize', 14, ...
        'FontWeight', 'bold', ...
        'LineWidth', 2.0, ...
        'Color', 'k');

    set(gca, ...
        'XTick', 1:ns, ...
        'YTick', 1:ns, ...
        'XTickLabel', state_names, ...
        'YTickLabel', state_names);

    title( ...
        'State correlation matrix at final step (P_{ij} / sqrt(P_{ii} P_{jj}))');

    %% ---------------- Innovation ----------------
    newfig('Innovation');

    inames = {'x', 'y', '\theta', 'v'};

    for i = 1:4

        subplot(2,2,i);

        iv = innov(ss,i);

        histogram(iv, 40, ...
            'Normalization', 'pdf', ...
            'FaceColor', [0.6 0.7 0.9], ...
            'EdgeColor', [0.2 0.2 0.2], ...
            'LineWidth', 1.3);
        hold on;

        s = std(iv);
        xx = linspace(min(iv), max(iv), 200);

        plot(xx, exp(-xx.^2/(2*s^2))/(s*sqrt(2*pi)), 'r-', ...
            'LineWidth', 3.2);

        grid on;

        title(sprintf( ...
            'innovation %s: var = %.4f', ...
            inames{i}, s^2));

    end

    if exist('sgtitle', 'file')

        sgtitle( ...
            ['Innovations z_k - H' x_hat_symbol '_k are zero-mean Gaussian'], ...
            'Interpreter', 'tex', ...
            'FontWeight', 'bold', ...
            'FontSize', 18, ...
            'Color', 'k');

    end

    drawnow;

    %% ---------------- Save figures ----------------
    if opts.Save
        save_tagged_figures(opts.SaveDir, opts.Prefix, tag);
    end

end

end

%% =====================================================================
%% Helpers
%% =====================================================================

function S = load_signals(src, names)
% Collect logged variables into a struct.
% src may be a SimulationOutput, a struct, or [].

    if isempty(src)

        if evalin('base', 'exist(''xhat_log'',''var'')')

            for i = 1:numel(names)
                S.(names{i}) = evalin('base', names{i});
            end

            return;

        end

        vars = evalin('base', 'whos');

        for iv = 1:numel(vars)

            if strcmp(vars(iv).class, 'Simulink.SimulationOutput')

                cand = evalin('base', vars(iv).name);

                try

                    S = load_signals(cand, names);

                    fprintf( ...
                        'Loaded logged signals from base-workspace ''%s''.\n', ...
                        vars(iv).name);

                    return;

                catch
                end

            end

        end

        error([ ...
            'No logged signals found. Pass the sim output, e.g. ', ...
            'plot_lkf_results(out), or make sure xhat_log etc. are in the ', ...
            'base workspace.']);

    end

    for i = 1:numel(names)

        if isa(src, 'Simulink.SimulationOutput')

            if ~any(strcmp(src.who, names{i}))

                error([ ...
                    'Sim output has no ''%s''. Check the To Workspace ', ...
                    'block variable name.'], names{i});

            end

            S.(names{i}) = src.get(names{i});

        elseif isstruct(src) && isfield(src, names{i})

            S.(names{i}) = src.(names{i});

        else

            error('Input has no ''%s''.', names{i});

        end

    end

end

function save_tagged_figures(folder, prefix, tag)
% Save every tagged figure as a 300 DPI PNG.

    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    figs = findobj('Type', 'figure', 'Tag', tag);

    [~, order] = sort([figs.Number]);
    figs = figs(order);

    for i = 1:numel(figs)

        name = figs(i).Name;

        if isempty(name)
            name = sprintf('figure%d', figs(i).Number);
        end

        name = regexprep(lower(name), '[^a-z0-9]+', '_');

        file = fullfile(folder, ...
            sprintf('%s_%02d_%s.png', prefix, i, name));

        if exist('exportgraphics', 'file')

            exportgraphics(figs(i), file, ...
                'Resolution', 300, ...
                'BackgroundColor', 'white');

        else

            print(figs(i), file, '-dpng', '-r300');

        end

    end

    fprintf('Saved %d figures to %s\n', numel(figs), folder);

end

function [A, tv] = grab(v, dt)
% N x n array and N x 1 time vector from a To Workspace variable.

    if isa(v, 'timeseries')

        A = squeeze(v.Data);
        tv = v.Time(:);

        if size(A,1) ~= numel(tv)
            A = A';
        end

    elseif isstruct(v) && isfield(v, 'signals')

        A = v.signals.values;

        if isfield(v, 'time') && ~isempty(v.time)
            tv = v.time(:);
        else
            tv = (0:size(A,1)-1)' * dt;
        end

    else

        A = v;
        tv = (0:size(A,1)-1)' * dt;

    end

end

function [A, tv] = grab3(v, dt)
% n x m x N array and N x 1 time vector for a matrix signal.

    if isa(v, 'timeseries')

        A = v.Data;
        tv = v.Time(:);

    elseif isstruct(v) && isfield(v, 'signals')

        A = v.signals.values;

        if isfield(v, 'time') && ~isempty(v.time)
            tv = v.time(:);
        else
            tv = (0:size(A,3)-1)' * dt;
        end

    else

        A = v;
        tv = (0:size(A,3)-1)' * dt;

    end

    if ndims(A) ~= 3

        error( ...
            'K_log / P_log must be logged as a matrix signal (n x m x N).');

    end

end

function B = align(A, tA, t)
% Resample rows of A onto the reference time grid using previous-sample hold.

    if numel(tA) == numel(t) && max(abs(tA(:) - t(:))) < 1e-9
        B = A;
        return;
    end

    [tA, iu] = unique(tA(:), 'stable');
    A = A(iu,:);

    B = interp1(tA, A, t, 'previous', 'extrap');

end

function B = align3(A, tA, t)
% Same as align, for n x m x N arrays.

    if numel(tA) == numel(t) && max(abs(tA(:) - t(:))) < 1e-9
        B = A;
        return;
    end

    [n, m, ~] = size(A);

    flat = reshape(A, n*m, []).';
    flat = align(flat, tA, t);

    B = reshape(flat.', n, m, []);

end