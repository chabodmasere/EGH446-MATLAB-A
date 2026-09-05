function res = param_estimation_constant(opts)
% PARAM_ESTIMATION_CONSTANT  Scalar KF estimating a constant from noisy data.
%
% Usage
%   param_estimation_constant                 metrics + figures (saved as PNG)
%   param_estimation_constant(Plots=false)    metrics only, no figure windows
%   res = param_estimation_constant(R_filter=0.1)   deliberate R mismatch
%   param_estimation_constant(P0=1e9)         diffuse prior -> exact running mean

arguments
    opts.c_true   (1,1) double  = 3.7     % the unknown constant
    opts.sigma    (1,1) double  = 0.8     % std dev of measurement noise (truth)
    opts.N        (1,1) double  = 200     % number of time steps
    opts.m        (1,1) double  = 1       % measurements per step
    opts.seed     (1,1) double  = 1       % rng seed, repeatable noise
    opts.c0       (1,1) double  = 0       % initial guess (deliberately wrong)
    opts.P0       (1,1) double  = 10      % initial uncertainty
    opts.R_filter (1,1) double  = NaN     % variance the FILTER is told; NaN = truth
    opts.Plots    (1,1) logical = true
    opts.Save     (1,1) logical = true
    opts.SaveDir  (1,:) char    = fullfile(fileparts(mfilename('fullpath')), 'figures')
    opts.Prefix   (1,:) char    = 'partA'
end

c_true = opts.c_true;
sigma = opts.sigma;
N = opts.N;
m = opts.m;

R_true = sigma^2;

if isnan(opts.R_filter)
    R_filt = R_true;
else
    R_filt = opts.R_filter;
end

mismatch = (R_filt ~= R_true);

tag = 'kf_partA';

set(0, 'DefaultFigureColor', 'w');

try
    set(groot, 'defaultFigureTheme', 'light');
catch
end

% Close only this function's old figures.
close(findobj('Type', 'figure', 'Tag', tag));

%% ---------------- Scenario ----------------
rng(opts.seed);

H = ones(m,1);                          % each sensor sees c directly
R = R_filt * eye(m);                    % what the filter believes
e = sigma * randn(m, N);                % the true Gaussian noise
Z = H * c_true + e;                     % m x N measurement sequence

%% ---------------- Kalman filter (recursive) ----------------
% State is the scalar c. A = 1, B = 0, Q = 0.
% Only the update step does any work.

c_hat   = zeros(1, N);
P       = zeros(1, N);
Kg      = zeros(m, N);
innov   = zeros(m, N);      % pre-update innovation
nu_norm = zeros(m, N);      % whitened innovation
S11     = zeros(1, N);      % innovation variance per component

c_prev = opts.c0;
P_prev = opts.P0;

for k = 1:N

    % Predict.
    c_pred = c_prev;
    P_pred = P_prev;

    % Innovation and its covariance.
    nu = Z(:,k) - H * c_pred;
    S  = H * P_pred * H' + R;

    % Kalman gain.
    K = P_pred * H' / S;

    % State update.
    c_prev = c_pred + K * nu;

    % Covariance update using the Joseph form.
    IKH = 1 - K * H;
    P_prev = IKH * P_pred * IKH' + K * R * K';

    % Store results.
    c_hat(k)     = c_prev;
    P(k)         = P_prev;
    Kg(:,k)      = K';
    innov(:,k)   = nu;
    S11(k)       = S(1,1);
    nu_norm(:,k) = chol(S, 'lower') \ nu;

end

%% ---------------- Diagnostics ----------------
err_sq = (c_hat - c_true).^2;

resid = Z - H * c_hat;
resid_v = resid(:);

nu_v = nu_norm(:);
n_nu = numel(nu_v);

running_mean = cumsum(mean(Z,1)) ./ (1:N);

% Closed-form claimed covariance:
% 1/P_N = 1/P0 + m*N/R.
P_theory = 1 / (1/opts.P0 + m*N/R_filt);

% Normalised innovation variance.
nu_var = var(nu_v);
nu_ci = 2 * sqrt(2 / n_nu);
nu_expect = 1;

if mismatch
    nu_expect = R_true / R_filt;
end

% Endpoint error relative to the filter's uncertainty.
consistency = abs(c_hat(end) - c_true) / sqrt(P(end));

% Correct-model prediction for posterior residual variance.
resid_var_theory = R_true - mean(P);
se_mean = sigma / sqrt(m*N);

fprintf('True constant           c     = %.4f\n', c_true);

fprintf('Final estimate          c_hat = %.4f\n', c_hat(end));

fprintf(['Final covariance        P_N   = %.4e   ' ...
         '(closed form 1/(1/P0+mN/R) = %.4e)\n'], ...
         P(end), P_theory);

fprintf(['Normalised innovation var     = %.4f   ' ...
         '(test: expect %.3f +/- %.3f, 95%%)\n'], ...
         nu_var, nu_expect, nu_ci);

fprintf(['Normalised innovation mean    = %.4f   ' ...
         '(expect 0 +/- %.3f, 95%%)\n'], ...
         mean(nu_v), 2/sqrt(n_nu));

fprintf(['Endpoint |err|/sqrt(P_N)      = %.2f   ' ...
         '(single draw of |N(0,1)|: E=0.80, wide spread)\n'], ...
         consistency);

fprintf('Noise variance (truth)  R     = %.4f\n', R_true);

if mismatch
    fprintf(['Noise variance (filter) R     = %.4g   ' ...
             '<-- mismatch on purpose\n'], R_filt);
end

fprintf(['Sample variance of residuals  = %.4f   ' ...
         '(theory: R - H*P*H'' = %.4f, just below R = %.2f)\n'], ...
         var(resid_v), resid_var_theory, R_true);

fprintf(['Sample mean of residuals      = %.4f   ' ...
         '(std error of this mean = %.4f)\n'], ...
         mean(resid_v), se_mean);

fprintf(['Max |KF estimate - running mean| = %.2e   ' ...
         '(prior''s fingerprint; -> 0 as P0 -> inf)\n'], ...
         max(abs(c_hat - running_mean)));

res = struct( ...
    'k', 1:N, ...
    'Z', Z, ...
    'c_hat', c_hat, ...
    'P', P, ...
    'K', Kg, ...
    'err_sq', err_sq, ...
    'resid', resid, ...
    'innov', innov, ...
    'nu_norm', nu_norm, ...
    'S', S11, ...
    'c_true', c_true, ...
    'R_true', R_true, ...
    'R_filter', R_filt, ...
    'P_theory', P_theory, ...
    'consistency', consistency, ...
    'nu_var', nu_var, ...
    'nu_ci', nu_ci, ...
    'nu_expect', nu_expect, ...
    'resid_var_theory', resid_var_theory, ...
    'running_mean', running_mean);

%% ---------------- Plots ----------------
if opts.Plots

    k = 1:N;

    % Bold styling for every figure and subplot.
    newfig = @(name) figure( ...
        'Name', name, ...
        'Tag', tag, ...
        'Color', 'w', ...
        'Units', 'pixels', ...
        'Position', [100 100 1200 760], ...
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

    % Unicode c-hat allows bold text with the TeX interpreter.
    c_hat_symbol = char(265);

    %% 1. Measured data
    newfig('Measured data');

    plot(k, Z', '.', ...
        'Color', [0.45 0.45 0.45], ...
        'MarkerSize', 12);
    hold on;

    plot([1 N], [c_true c_true], 'r-', ...
        'LineWidth', 3.2);

    xlabel('k');
    ylabel('z_k');
    grid on;

    title(sprintf( ...
        'Measured data  z_k = c + e_k,  e_k ~ N(0, %.2f)', ...
        R_true));

    legend('measurements', 'true c', ...
        'Location', 'best');

    %% 2. Recursive estimate vs true constant
    newfig('Recursive estimate');

    plot(k, Z', '.', ...
        'Color', [0.55 0.55 0.55], ...
        'MarkerSize', 12);
    hold on;

    plot(k, c_hat, 'b-', ...
        'LineWidth', 3.2);

    plot([1 N], [c_true c_true], 'r-', ...
        'LineWidth', 3.2);

    xlabel('k');
    ylabel('c');
    grid on;

    title('Recursive Kalman estimate of the constant');

    legend( ...
        {'measurements', ['estimate ' c_hat_symbol '_k'], 'true c'}, ...
        'Interpreter', 'tex', ...
        'Location', 'best');

    %% 3. Squared error vs covariance
    newfig('Estimate error squared');

    semilogy(k, err_sq, 'b-', ...
        'LineWidth', 3.0);
    hold on;

    semilogy(k, P, 'r--', ...
        'LineWidth', 3.0);

    xlabel('k');
    grid on;

    if mismatch
        title({ ...
            'Single-run squared error vs. filter''s ensemble variance P_k', ...
            sprintf( ...
                'filter told R = %.2g, truth R = %.2g (mismatch)', ...
                R_filt, R_true)});
    else
        title({ ...
            'Single-run squared error vs. filter''s ensemble variance P_k', ...
            'one realisation should straddle P_k, not coincide with it'});
    end

    legend( ...
        {['(' c_hat_symbol '_k - c)^2 (this run)'], ...
         'P_k (ensemble variance claimed by filter)'}, ...
        'Interpreter', 'tex', ...
        'Location', 'best');

    %% 4. Kalman gain and covariance
    newfig('Kalman gain and covariance');

    subplot(2,1,1);

    plot(k, Kg', 'b-', ...
        'LineWidth', 3.0);

    grid on;
    ylabel('K_k');

    title('Kalman gain - decays like 1/k for a constant (Q = 0)');

    subplot(2,1,2);

    semilogy(k, P, 'b-', ...
        'LineWidth', 3.0);
    hold on;

    semilogy(k, P_theory * ones(1,N), 'k:', ...
        'LineWidth', 3.0);

    xlabel('k');
    ylabel('P_k');
    grid on;

    title('Error covariance');

    legend('P_k', 'closed-form P_N', ...
        'Location', 'best');

    %% 5. Innovations and normalised innovation distribution
    newfig('Observed noise');

    subplot(2,1,1);

    plot(k, innov', '.', ...
        'Color', [0.3 0.3 0.8], ...
        'MarkerSize', 12);
    hold on;

    plot(k, 2*sqrt(S11), 'r--', ...
        'LineWidth', 3.0);

    plot(k, -2*sqrt(S11), 'r--', ...
        'LineWidth', 3.0);

    xlabel('k');

    ylabel( ...
        ['\nu_k = z_k - ' c_hat_symbol '_{k|k-1}'], ...
        'Interpreter', 'tex');

    grid on;

    title( ...
        'Innovations with \pm2\surdS_k envelope (~95% should fall inside)');

    subplot(2,1,2);

    histogram(nu_v, 30, ...
        'Normalization', 'pdf', ...
        'FaceColor', [0.6 0.7 0.9], ...
        'EdgeColor', [0.2 0.2 0.2], ...
        'LineWidth', 1.3);
    hold on;

    xx = linspace(min(nu_v), max(nu_v), 200);

    plot(xx, exp(-xx.^2/2)/sqrt(2*pi), 'r-', ...
        'LineWidth', 3.2);

    xlabel('normalised innovation \nu_k / \surdS_k');
    ylabel('pdf');
    grid on;

    title(sprintf( ...
        'Normalised innovations: var = %.3f  (expect %.3f \\pm %.3f)', ...
        nu_var, nu_expect, nu_ci));

    legend('observed', 'N(0, 1)', ...
        'Location', 'best');

    drawnow;

    %% Save figures
    if opts.Save
        save_tagged_figures(opts.SaveDir, opts.Prefix, tag);
    end

end

end

%% ---------------- Helper ----------------
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