function stats = plot_ekf_results(src, opts)

arguments
    src = []
    opts.dt      (1,1) double  = 0.01
    opts.Save    (1,1) logical = true
    opts.SaveDir (1,:) char    = fullfile(fileparts(mfilename('fullpath')), 'figures')
    opts.Prefix  (1,:) char    = 'partB'
end

dt  = opts.dt;
tag = 'ekf_partB';

set(0, 'DefaultFigureColor', 'w');
try, set(groot, 'defaultFigureTheme', 'light'); catch, end   % R2025a+

% Close only figures from a previous run of THIS function (not the animation).
close(findobj('Type', 'figure', 'Tag', tag));

%% ---------------- Load logged signals ----------------
names = {'pose_log','z_log','xhat_log','K_log','P_log','innov_log'};
S = load_signals(src, names);

[xhat,  t_x] = grab(S.xhat_log,  dt);    % reference grid: the filter's output
[pose,  t_p] = grab(S.pose_log,  dt);
[z,     t_z] = grab(S.z_log,     dt);
[innov, t_i] = grab(S.innov_log, dt);
[K,     t_K] = grab3(S.K_log,    dt);
[P,     t_P] = grab3(S.P_log,    dt);

t     = t_x(:);
pose  = align(pose,  t_p, t);
z     = align(z,     t_z, t);
innov = align(innov, t_i, t);
K     = align3(K, t_K, t);
P     = align3(P, t_P, t);

N = numel(t);
k = (1:N)';

if abs(numel(t_p) - N) > 1
    fprintf(['Note: pose_log had %d samples but xhat_log had %d. They were ', ...
             'resampled onto a common grid. Set every To Workspace block to ', ...
             'Sample time 0.01 to avoid this.\n\n'], numel(t_p), N);
end

wrap = @(a) atan2(sin(a), cos(a));

%% ---------------- Errors ----------------
err           = pose - xhat;
err(:,3)      = wrap(err(:,3));
meas_err      = pose - z(:,1:3);
meas_err(:,3) = wrap(meas_err(:,3));
err_sq        = err.^2;

rmse_est  = sqrt(mean(err.^2));
rmse_meas = sqrt(mean(meas_err.^2));

fprintf('              x         y        theta\n');
fprintf('RMSE raw    %8.4f  %8.4f  %8.4f\n', rmse_meas);
fprintf('RMSE EKF    %8.4f  %8.4f  %8.4f\n', rmse_est);
fprintf('Improvement %8.2fx %8.2fx %8.2fx\n', rmse_meas ./ rmse_est);
fprintf('\nSteady-state covariance P (last step):\n');  disp(P(:,:,end));
fprintf('Steady-state Kalman gain K (last step):\n');   disp(K(:,:,end));
fprintf('Innovation variance (should be near R + H P H''):\n'); disp(var(innov));

stats = struct('t', t, 'err', err, 'meas_err', meas_err, ...
               'rmse_est', rmse_est, 'rmse_meas', rmse_meas, ...
               'P_ss', P(:,:,end), 'K_ss', K(:,:,end), 'innov_var', var(innov));

newfig = @(name) figure('Name', name, 'Tag', tag);

%% ---------------- Fig 1: states against k ----------------
labels = {'x  [m]', 'y  [m]', '\theta  [rad]'};
newfig('State estimates vs k');
pose_p = pose;  z_p = z;
pose_p(:,3) = wrap(pose_p(:,3));  z_p(:,3) = wrap(z_p(:,3));
for i = 1:3
    subplot(3,1,i);
    plot(k, z_p(:,i), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3); hold on;
    plot(k, pose_p(:,i), 'r-', 'LineWidth', 1.4);
    plot(k, xhat(:,i), 'b-', 'LineWidth', 1.0);
    ylabel(labels{i}); grid on;
    if i == 1, title('Measured data, true state (red) and EKF estimate (blue)'); end
    if i == 3, ylabel('\theta  [rad, wrapped]'); xlabel('k'); end
end
legend('measured', 'true', 'estimate', 'Location', 'best');

%% ---------------- Fig 2: x-y trajectory ----------------
newfig('x-y trajectory');
plot(z(:,1), z(:,2), '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3); hold on;
plot(pose(:,1), pose(:,2), 'r-', 'LineWidth', 2);
plot(xhat(:,1), xhat(:,2), 'b-', 'LineWidth', 1.2);
axis equal; grid on;
xlabel('x  [m]'); ylabel('y  [m]');
title('Vehicle path: measured (grey), true (red), estimated (blue)');
legend('measured', 'true', 'estimate', 'Location', 'best');

%% ---------------- Fig 3: error and error squared ----------------
newfig('Estimation error');
subplot(2,1,1);
plot(k, err(:,1), 'b-', k, err(:,2), 'c-', k, err(:,3), 'm-'); grid on;
ylabel('error'); title('Estimation error  $x_k - \hat{x}_k$', 'Interpreter', 'latex');
legend('x', 'y', '\theta', 'Location', 'best');
subplot(2,1,2);
semilogy(k, err_sq(:,1), 'b-', k, err_sq(:,2), 'c-', k, err_sq(:,3), 'm-'); grid on;
xlabel('k'); ylabel('error^2'); title('Estimate error squared');

newfig('EKF vs raw sensor');
subplot(2,1,1);
plot(k, meas_err(:,1), '-', 'Color', [0.7 0.7 0.7]); hold on;
plot(k, err(:,1), 'b-'); grid on;
ylabel('x error [m]'); title('Raw measurement error (grey) vs EKF error (blue)');
subplot(2,1,2);
plot(k, meas_err(:,3), '-', 'Color', [0.7 0.7 0.7]); hold on;
plot(k, err(:,3), 'b-'); grid on;
xlabel('k'); ylabel('\theta error [rad]');

%% ---------------- Fig 4: Kalman gain ----------------
kz = min(300, N);
newfig('Kalman gain');
subplot(3,1,1);
plot(k, squeeze(K(1,1,:)), 'b-', k, squeeze(K(2,2,:)), 'c-', ...
     k, squeeze(K(3,3,:)), 'm-', k, squeeze(K(4,4,:)), 'g-'); grid on;
ylabel('gain'); title('Diagonal Kalman gains, full run: converge to steady state and stay there');
legend('K_{x,x}', 'K_{y,y}', 'K_{\theta,\theta}', 'K_{v,v}', 'Location', 'best');
subplot(3,1,2);
plot(k(1:kz), squeeze(K(1,1,1:kz)), 'b-', k(1:kz), squeeze(K(2,2,1:kz)), 'c-', ...
     k(1:kz), squeeze(K(3,3,1:kz)), 'm-', k(1:kz), squeeze(K(4,4,1:kz)), 'g-'); grid on;
ylabel('gain'); title(sprintf('Same gains, first %d steps: large while P is large, then settle', kz));
subplot(3,1,3);
plot(k(1:kz), squeeze(K(5,3,1:kz)), 'm-', k(1:kz), squeeze(K(1,3,1:kz)), 'b-'); grid on;
xlabel('k'); ylabel('gain');
title('Cross gains, first steps: heading measurement -> \omega and x');
legend('K_{\omega,\theta}', 'K_{x,\theta}', 'Location', 'best');

%% ---------------- Fig 5: covariance ----------------
newfig('Error covariance');
Pd = zeros(N,5);
for i = 1:5, Pd(:,i) = squeeze(P(i,i,:)); end
subplot(2,1,1);
semilogy(k, Pd, 'LineWidth', 1.1); grid on;
ylabel('P_{ii}'); title('Diagonal of P, full run: filter''s own variance for each state');
legend('x', 'y', '\theta', 'v', '\omega', 'Location', 'best');
subplot(2,1,2);
semilogy(k(1:kz), Pd(1:kz,:), 'LineWidth', 1.1); grid on;
xlabel('k'); ylabel('P_{ii}');
title(sprintf('First %d steps: uncertainty collapses from the initial guess to steady state', kz));

newfig('Steady-state covariance');
imagesc(P(:,:,end)); colorbar; axis square;
set(gca, 'XTick', 1:5, 'YTick', 1:5, ...
    'XTickLabel', {'x','y','\theta','v','\omega'}, ...
    'YTickLabel', {'x','y','\theta','v','\omega'});
title('Full-state covariance matrix P at final step');

%% ---------------- Fig 6: innovation ----------------
newfig('Innovation');
inames = {'x', 'y', '\theta', 'v'};
for i = 1:4
    subplot(2,2,i);
    histogram(innov(:,i), 40, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9]); hold on;
    s  = std(innov(:,i));
    xx = linspace(min(innov(:,i)), max(innov(:,i)), 200);
    plot(xx, exp(-xx.^2/(2*s^2))/(s*sqrt(2*pi)), 'r-', 'LineWidth', 1.4);
    grid on; title(sprintf('innovation %s: var = %.4f', inames{i}, s^2));
end
if exist('sgtitle','file')
    sgtitle('Innovations $z_k - H\hat{x}_k$ are zero-mean Gaussian', 'Interpreter', 'latex');
end

drawnow;

%% ---------------- Save every tagged figure as PNG ----------------
if opts.Save
    save_tagged_figures(opts.SaveDir, opts.Prefix, tag);
end

end   % plot_ekf_results

%% =====================================================================
%% helpers
%% =====================================================================
function S = load_signals(src, names)
% Collect the logged variables into a struct S with fields named in `names`.
% src may be a Simulink.SimulationOutput, a struct with those fields, or [].
% If [] : look in the base workspace for the loose variables, or for any
%         SimulationOutput object that holds them.
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
                    fprintf('Loaded logged signals from base-workspace ''%s''.\n', vars(iv).name);
                    return;
                catch
                end
            end
        end
        error(['No logged signals found. Pass the sim output, e.g. ', ...
               'plot_ekf_results(out), or make sure xhat_log etc. are in the ', ...
               'base workspace.']);
    end

    for i = 1:numel(names)
        if isa(src, 'Simulink.SimulationOutput')
            if ~any(strcmp(src.who, names{i}))
                error('Sim output has no ''%s''. Check the To Workspace block variable name.', names{i});
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
% Save every figure with the given Tag to folder/<prefix>_<nn>_<name>.png.
    if ~exist(folder, 'dir'), mkdir(folder); end
    figs = findobj('Type', 'figure', 'Tag', tag);
    [~, order] = sort([figs.Number]);
    figs = figs(order);
    for i = 1:numel(figs)
        name = figs(i).Name;
        if isempty(name), name = sprintf('figure%d', figs(i).Number); end
        name = regexprep(lower(name), '[^a-z0-9]+', '_');
        file = fullfile(folder, sprintf('%s_%02d_%s.png', prefix, i, name));
        if exist('exportgraphics', 'file')
            exportgraphics(figs(i), file, 'Resolution', 200);
        else
            print(figs(i), file, '-dpng', '-r200');
        end
    end
    fprintf('Saved %d figures to %s\n', numel(figs), folder);
end

function [A, tv] = grab(v, dt)
% N x n array and N x 1 time vector from a To Workspace variable.
    if isa(v, 'timeseries')
        A  = squeeze(v.Data);
        tv = v.Time(:);
        if size(A,1) ~= numel(tv), A = A'; end
    elseif isstruct(v) && isfield(v, 'signals')
        A  = v.signals.values;
        if isfield(v, 'time') && ~isempty(v.time)
            tv = v.time(:);
        else
            tv = (0:size(A,1)-1)' * dt;
        end
    else
        A  = v;
        tv = (0:size(A,1)-1)' * dt;
    end
end

function [A, tv] = grab3(v, dt)
% n x m x N array and N x 1 time vector for a matrix signal.
    if isa(v, 'timeseries')
        A  = v.Data;
        tv = v.Time(:);
    elseif isstruct(v) && isfield(v, 'signals')
        A  = v.signals.values;
        if isfield(v, 'time') && ~isempty(v.time)
            tv = v.time(:);
        else
            tv = (0:size(A,3)-1)' * dt;
        end
    else
        A  = v;
        tv = (0:size(A,3)-1)' * dt;
    end
    if ndims(A) ~= 3
        error('K_log / P_log must be logged as a matrix signal (n x m x N).');
    end
end

function B = align(A, tA, t)
% Resample rows of A (sampled at tA) onto time grid t. Nearest-sample hold.
    if numel(tA) == numel(t) && max(abs(tA(:) - t(:))) < 1e-9
        B = A; return;
    end
    [tA, iu] = unique(tA(:), 'stable');
    A = A(iu, :);
    B = interp1(tA, A, t, 'previous', 'extrap');
end

function B = align3(A, tA, t)
% Same as align, for n x m x N arrays.
    if numel(tA) == numel(t) && max(abs(tA(:) - t(:))) < 1e-9
        B = A; return;
    end
    [n, m, ~] = size(A);
    flat = reshape(A, n*m, []).';
    flat = align(flat, tA, t);
    B = reshape(flat.', n, m, []);
end