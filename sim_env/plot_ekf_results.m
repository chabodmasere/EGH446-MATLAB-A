%% PART B - plots for the EKF state estimator
%
% Run the Simulink model first with these To Workspace blocks inside the
% Estimator subsystem (Save format: "Array" or "Timeseries", both handled):
%
%   Variable name   Wired to                         Size per step
%   pose_log        pose inport (truth)              3
%   z_log           Mux output z                     4
%   xhat_log        ekf_estimator x_hat              3
%   K_log           ekf_estimator K                  5x4
%   P_log           ekf_estimator P_out              5x5
%   innov_log       ekf_estimator innov              4
%
% All To Workspace blocks: Sample time = 0.01 (same as the filter).
% Prefer Save format = Timeseries: it stores a time stamp with every sample
% so this script can align the signals even if a sample time was left at -1.
% Array format has no time stamps, so with Array every block MUST be 0.01.
%
% Colour convention: blue solid = estimate, red = true, grey = measurement.

clear pose z xhat K P innov; clc; close all;
set(0, 'DefaultFigureColor', 'w');   % white figure backgrounds for the report
try, set(groot, 'defaultFigureTheme', 'light'); catch, end   % R2025a+: force light axes too

dt = 0.01;

% Find the logged signals. They may be loose in the workspace, or bundled
% inside a Simulink.SimulationOutput object (default name 'out', but the
% sim environment may use another name). Search for any such object.
if ~exist('xhat_log','var')
    vars = whos;
    names = {'pose_log','z_log','xhat_log','K_log','P_log','innov_log'};
    found = false;
    for iv = 1:numel(vars)
        if strcmp(vars(iv).class, 'Simulink.SimulationOutput') || ...
           (strcmp(vars(iv).class, 'struct') && isfield(eval(vars(iv).name), 'xhat_log'))
            so = eval(vars(iv).name);
            try
                for in = 1:numel(names)
                    assignin('base', names{in}, so.(names{in}));
                    eval([names{in} ' = so.(names{in});']);
                end
                found = true;
                fprintf('Loaded logged signals from ''%s''.\n', vars(iv).name);
                break;
            catch
            end
        end
    end
    if ~found
        error(['No logged signals found. After running the model, the workspace ', ...
               'must contain xhat_log etc., or a Simulink.SimulationOutput holding ', ...
               'them. Type  whos  to see what the simulation produced.']);
    end
end

% Load each signal together with its own time vector, then put everything
% on the filter's 0.01 s grid. This makes the comparison correct even if
% some To Workspace blocks were left at an inherited (-1) sample time.
[xhat,  t_x]  = grab(xhat_log,  dt);    % reference grid: the filter's output
[pose,  t_p]  = grab(pose_log,  dt);
[z,     t_z]  = grab(z_log,     dt);
[innov, t_i]  = grab(innov_log, dt);
[K,     t_K]  = grab3(K_log,    dt);
[P,     t_P]  = grab3(P_log,    dt);

t     = t_x(:);
pose  = align(pose,  t_p, t);
z     = align(z,     t_z, t);
innov = align(innov, t_i, t);
K     = align3(K, t_K, t);
P     = align3(P, t_P, t);

N = numel(t);
k = (1:N)';                 % discrete time index

if abs(numel(t_p) - N) > 1
    fprintf(['Note: pose_log had %d samples but xhat_log had %d. They were ', ...
             'resampled onto a common grid. Set every To Workspace block to ', ...
             'Sample time 0.01 to avoid this.\n\n'], numel(t_p), N);
end

wrap = @(a) atan2(sin(a), cos(a));

%% ---------------- Errors ----------------
err          = pose - xhat;                 % estimation error
err(:,3)     = wrap(err(:,3));              % angle error must be wrapped
meas_err     = pose - z(:,1:3);             % raw sensor error, for comparison
meas_err(:,3)= wrap(meas_err(:,3));
err_sq       = err.^2;

rmse_est  = sqrt(mean(err.^2));
rmse_meas = sqrt(mean(meas_err.^2));
fprintf('              x         y        theta\n');
fprintf('RMSE raw    %8.4f  %8.4f  %8.4f\n', rmse_meas);
fprintf('RMSE EKF    %8.4f  %8.4f  %8.4f\n', rmse_est);
fprintf('Improvement %8.2fx %8.2fx %8.2fx\n', rmse_meas ./ rmse_est);
fprintf('\nSteady-state covariance P (last step):\n');
disp(P(:,:,end));
fprintf('Steady-state Kalman gain K (last step):\n');
disp(K(:,:,end));
fprintf('Innovation variance (should be near R + H P H''):\n');
disp(var(innov));

%% ---------------- Fig 1: states against k ----------------
labels = {'x  [m]', 'y  [m]', '\theta  [rad]'};
figure('Name','State estimates vs k');
pose_p = pose;  z_p = z;                       % wrapped copies for plotting only
pose_p(:,3) = wrap(pose_p(:,3));  z_p(:,3) = wrap(z_p(:,3));
for i = 1:3
    subplot(3,1,i);
    plot(k, z_p(:,i), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3); hold on;
    plot(k, pose_p(:,i), 'r-', 'LineWidth', 1.4);
    plot(k, xhat(:,i), 'b-', 'LineWidth', 1.0);
    ylabel(labels{i}); grid on;
    if i == 1, title('Measured data, true state (red) and EKF estimate (blue)'); end
    if i == 3, ylabel('\theta  [rad, wrapped]'); end
    if i == 3, xlabel('k'); end
end
legend('measured', 'true', 'estimate', 'Location', 'best');

%% ---------------- Fig 2: x-y trajectory ----------------
figure('Name','x-y trajectory');
plot(z(:,1), z(:,2), '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3); hold on;
plot(pose(:,1), pose(:,2), 'r-', 'LineWidth', 2);
plot(xhat(:,1), xhat(:,2), 'b-', 'LineWidth', 1.2);
axis equal; grid on;
xlabel('x  [m]'); ylabel('y  [m]');
title('Vehicle path: measured (grey), true (red), estimated (blue)');
legend('measured', 'true', 'estimate', 'Location', 'best');

%% ---------------- Fig 3: error and error squared ----------------
figure('Name','Estimation error');
subplot(2,1,1);
plot(k, err(:,1), 'b-', k, err(:,2), 'c-', k, err(:,3), 'm-'); grid on;
ylabel('error'); title('Estimation error  $x_k - \hat{x}_k$', 'Interpreter', 'latex');
legend('x', 'y', '\theta', 'Location', 'best');
subplot(2,1,2);
semilogy(k, err_sq(:,1), 'b-', k, err_sq(:,2), 'c-', k, err_sq(:,3), 'm-'); grid on;
xlabel('k'); ylabel('error^2'); title('Estimate error squared');

figure('Name','EKF vs raw sensor');
subplot(2,1,1);
plot(k, meas_err(:,1), '-', 'Color', [0.7 0.7 0.7]); hold on;
plot(k, err(:,1), 'b-'); grid on;
ylabel('x error [m]'); title('Raw measurement error (grey) vs EKF error (blue)');
subplot(2,1,2);
plot(k, meas_err(:,3), '-', 'Color', [0.7 0.7 0.7]); hold on;
plot(k, err(:,3), 'b-'); grid on;
xlabel('k'); ylabel('\theta error [rad]');

%% ---------------- Fig 4: Kalman gain ----------------
% K(i,j): how much state i moves per unit innovation in measurement j.
kz = min(300, N);                                   % zoom window for the transient
figure('Name','Kalman gain');
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
figure('Name','Error covariance');
Pd = zeros(N,5);
for i = 1:5, Pd(:,i) = squeeze(P(i,i,:)); end
subplot(2,1,1);
semilogy(k, Pd, 'LineWidth', 1.1); grid on;
ylabel('P_{ii}');
title('Diagonal of P, full run: filter''s own variance for each state');
legend('x', 'y', '\theta', 'v', '\omega', 'Location', 'best');
subplot(2,1,2);
semilogy(k(1:kz), Pd(1:kz,:), 'LineWidth', 1.1); grid on;
xlabel('k'); ylabel('P_{ii}');
title(sprintf('First %d steps: uncertainty collapses from the initial guess to steady state', kz));

figure('Name','Steady-state covariance');
imagesc(P(:,:,end)); colorbar; axis square;
set(gca, 'XTick', 1:5, 'YTick', 1:5, ...
    'XTickLabel', {'x','y','\theta','v','\omega'}, ...
    'YTickLabel', {'x','y','\theta','v','\omega'});
title('Full-state covariance matrix P at final step');

%% ---------------- Fig 6: innovation (observed noise) ----------------
figure('Name','Innovation');
names = {'x', 'y', '\theta', 'v'};
for i = 1:4
    subplot(2,2,i);
    histogram(innov(:,i), 40, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9]); hold on;
    s  = std(innov(:,i));
    xx = linspace(min(innov(:,i)), max(innov(:,i)), 200);
    plot(xx, exp(-xx.^2/(2*s^2))/(s*sqrt(2*pi)), 'r-', 'LineWidth', 1.4);
    grid on; title(sprintf('innovation %s: var = %.4f', names{i}, s^2));
end
if exist('sgtitle','file'), sgtitle('Innovations $z_k - H\hat{x}_k$ are zero-mean Gaussian', 'Interpreter', 'latex'); end

%% ---------------- Save every figure as PNG ----------------
save_all_figures(fullfile(fileparts(mfilename('fullpath')), 'figures'), 'partB');

%% ---------------- helpers ----------------
function save_all_figures(folder, prefix)
% Save every open figure to folder/<prefix>_<nn>_<name>.png at 200 dpi.
    if ~exist(folder, 'dir'), mkdir(folder); end
    figs = findobj('Type', 'figure');
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
% Return N x n array and its N x 1 time vector from a To Workspace variable.
% Handles Timeseries, Structure With Time, and plain Array formats.
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
        tv = (0:size(A,1)-1)' * dt;   % Array format has no time: assume dt
    end
end

function [A, tv] = grab3(v, dt)
% Return n x m x N array and N x 1 time vector for a matrix signal.
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
    flat = reshape(A, n*m, []).';            % N x (n*m)
    flat = align(flat, tA, t);
    B = reshape(flat.', n, m, []);
end