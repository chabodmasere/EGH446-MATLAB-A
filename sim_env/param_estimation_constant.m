function res = param_estimation_constant(opts)

arguments
    opts.c_true   (1,1) double  = 3.7     % the unknown constant
    opts.sigma    (1,1) double  = 0.8     % std dev of measurement noise (truth)
    opts.N        (1,1) double  = 200     % number of time steps
    opts.m        (1,1) double  = 1       % measurements per step
    opts.seed     (1,1) double  = 1       % rng seed, repeatable noise
    opts.c0       (1,1) double  = 0       % initial guess (deliberately wrong)
    opts.P0       (1,1) double  = 10      % initial uncertainty
    opts.R_filter (1,1) double  = NaN     % variance the FILTER is told; NaN = truth
    opts.Save     (1,1) logical = true
    opts.SaveDir  (1,:) char    = fullfile(fileparts(mfilename('fullpath')), 'figures')
    opts.Prefix   (1,:) char    = 'partA'
end

c_true = opts.c_true;  sigma = opts.sigma;  N = opts.N;  m = opts.m;
R_true = sigma^2;
if isnan(opts.R_filter), R_filt = R_true; else, R_filt = opts.R_filter; end

tag = 'kf_partA';
set(0, 'DefaultFigureColor', 'w');
try, set(groot, 'defaultFigureTheme', 'light'); catch, end
close(findobj('Type', 'figure', 'Tag', tag));       % only this function's old figures

%% ---------------- Scenario ----------------
rng(opts.seed);
H = ones(m,1);                          % each sensor sees c directly
R = R_filt * eye(m);                    % what the filter believes
e = sigma * randn(m, N);                % the true Gaussian noise
Z = H * c_true + e;                     % m x N measurement sequence

%% ---------------- Kalman filter (recursive) ----------------
% State is the scalar c. A = 1, B = 0, Q = 0 (a true constant has no
% process noise). Only the update step does any work.
c_hat = zeros(1, N);
P     = zeros(1, N);
Kg    = zeros(m, N);

c_prev = opts.c0;
P_prev = opts.P0;

for k = 1:N
    % Predict (trivial for a constant)
    c_pred = c_prev;                        % A*c, A = 1
    P_pred = P_prev;                        % A*P*A' + Q, Q = 0

    % Innovation and its covariance
    innov = Z(:,k) - H * c_pred;            % m x 1
    S     = H * P_pred * H' + R;            % m x m

    % Gain: how much of the innovation to believe
    K = P_pred * H' / S;                    % 1 x m

    % Update
    c_prev = c_pred + K * innov;
    P_prev = (1 - K * H) * P_pred;

    c_hat(k) = c_prev;
    P(k)     = P_prev;
    Kg(:,k)  = K';
end

%% ---------------- Diagnostics ----------------
err_sq  = (c_hat - c_true).^2;
resid   = Z - H * c_hat;                    % observed noise, m x N
resid_v = resid(:);
running_mean = cumsum(mean(Z,1)) ./ (1:N);  % KF for a constant = running mean

fprintf('True constant           c     = %.4f\n', c_true);
fprintf('Final estimate          c_hat = %.4f\n', c_hat(end));
fprintf('Final covariance        P_N   = %.4e\n', P(end));
fprintf('Noise variance (truth)  R     = %.4f\n', R_true);
if R_filt ~= R_true
    fprintf('Noise variance (filter) R     = %.4g   <-- mismatch on purpose\n', R_filt);
end
fprintf('Sample variance of residuals  = %.4f   (should be close to R)\n', var(resid_v));
fprintf('Sample mean of residuals      = %.4f   (should be close to 0)\n', mean(resid_v));
fprintf('Max |KF estimate - running mean| = %.2e\n', max(abs(c_hat - running_mean)));

res = struct('k', 1:N, 'Z', Z, 'c_hat', c_hat, 'P', P, 'K', Kg, ...
             'err_sq', err_sq, 'resid', resid, 'c_true', c_true, ...
             'R_true', R_true, 'R_filter', R_filt, 'running_mean', running_mean);

%% ---------------- Plots ----------------
k = 1:N;
newfig = @(name) figure('Name', name, 'Tag', tag);

% 1. Measured data
newfig('Measured data');
plot(k, Z', '.', 'Color', [0.5 0.5 0.5]); hold on;
plot([1 N], [c_true c_true], 'r-', 'LineWidth', 1.5);
xlabel('k'); ylabel('z_k'); grid on;
title(sprintf('Measured data  z_k = c + e_k,  e_k ~ N(0, %.2f)', R_true));
legend('measurements', 'true c', 'Location', 'best');

% 2. Recursive estimate vs true
newfig('Recursive estimate');
plot(k, Z', '.', 'Color', [0.75 0.75 0.75]); hold on;
plot(k, c_hat, 'b-', 'LineWidth', 1.5);
plot([1 N], [c_true c_true], 'r-', 'LineWidth', 1.5);
xlabel('k'); ylabel('c'); grid on;
title('Recursive Kalman estimate of the constant');
legend({'measurements', 'estimate $\hat{c}_k$', 'true $c$'}, ...
       'Interpreter', 'latex', 'Location', 'best');

% 3. Error squared
newfig('Estimate error squared');
semilogy(k, err_sq, 'b-', 'LineWidth', 1.2); hold on;
semilogy(k, P, 'r--', 'LineWidth', 1.2);
xlabel('k'); grid on;
title('Estimate error squared vs. filter covariance');
legend({'$(\hat{c}_k - c)^2$', '$P_k$ (filter''s own error variance)'}, ...
       'Interpreter', 'latex', 'Location', 'best');

% 4. Gain and covariance
newfig('Kalman gain and covariance');
subplot(2,1,1);
plot(k, Kg', 'b-', 'LineWidth', 1.2); grid on;
ylabel('K_k'); title('Kalman gain - decays like 1/k for a constant');
subplot(2,1,2);
semilogy(k, P, 'b-', 'LineWidth', 1.2); grid on;
xlabel('k'); ylabel('P_k'); title('Error covariance');

% 5. Observed Gaussian noise
newfig('Observed noise');
subplot(2,1,1);
plot(k, resid', '.', 'Color', [0.3 0.3 0.8]); hold on;
plot([1 N], [0 0], 'r-');
xlabel('k'); ylabel('$z_k - \hat{c}_k$', 'Interpreter', 'latex'); grid on;
title('Residuals (observed measurement noise)');
subplot(2,1,2);
histogram(resid_v, 30, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9]); hold on;
xx = linspace(min(resid_v), max(resid_v), 200);
gauss = exp(-xx.^2 / (2*sigma^2)) / (sigma*sqrt(2*pi));
plot(xx, gauss, 'r-', 'LineWidth', 1.5);
xlabel('residual'); ylabel('pdf'); grid on;
title(sprintf('Zero-mean Gaussian, variance = %.3f  (R = %.3f)', var(resid_v), R_true));
legend('observed', 'N(0, R)', 'Location', 'best');

drawnow;

%% ---------------- Save every tagged figure as PNG ----------------
if opts.Save
    save_tagged_figures(opts.SaveDir, opts.Prefix, tag);
end

end   % param_estimation_constant

%% ---------------- helpers ----------------
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