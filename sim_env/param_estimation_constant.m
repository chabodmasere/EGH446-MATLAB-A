%% PART A - Kalman filter for parameter estimation (unknown constant c)
%
% Process:      c_{k+1} = c_k                 (the parameter does not change)
% Measurement:  z_k     = H*c_k + e_k,  e_k ~ N(0, R)   (zero-mean Gaussian)
%
% With m sensors per step, z_k is an m x 1 vector and H = ones(m,1), i.e.
% every measurement is "the constant plus its own noise". m = 1 gives the
% scalar case.
%
% Plots produced (blue solid = estimate, red = true value):
%   1. measured data z_k against k
%   2. recursive estimate c_hat_k against k, with true c
%   3. estimate error squared (c_hat_k - c)^2 against k
%   4. Kalman gain K_k and error covariance P_k against k
%   5. residuals z_k - c_hat_k : the observed Gaussian noise, with histogram
%
% Run this script on its own - it does not need the Simulink model.

clearvars -except *_log out; clc; close all;   % keep Simulink logs alive
set(0, 'DefaultFigureColor', 'w');   % white figure backgrounds for the report

%% ---------------- Scenario ----------------
c_true = 3.7;          % the unknown constant we are trying to find
sigma  = 0.8;          % std dev of measurement noise
R_true = sigma^2;      % its variance (what the filter is told as R)
N      = 200;          % number of time steps
m      = 1;            % measurements per step (1 = scalar, 3 = vector of 3 sensors)
rng(1);                % repeatable noise

H = ones(m,1);                          % each sensor sees c directly
R = R_true * eye(m);                    % measurement covariance
e = sigma * randn(m, N);                % the Gaussian noise sequence
Z = H * c_true + e;                     % m x N measurement sequence

%% ---------------- Kalman filter (recursive) ----------------
% State is the scalar c. A = 1, B = 0, Q = 0 (a true constant has no
% process noise). Only the update step does any work.

c_hat = zeros(1, N);   % estimate after each measurement
P     = zeros(1, N);   % error covariance after each measurement
Kg    = zeros(m, N);   % Kalman gain (m x 1 each step)

c_prev = 0;            % initial guess (deliberately wrong)
P_prev = 10;           % large initial uncertainty = "I don't know c"

for k = 1:N
    % Predict (trivial for a constant)
    c_pred = c_prev;           % A*c, with A = 1
    P_pred = P_prev;           % A*P*A' + Q, with Q = 0

    % Innovation: how far the measurement is from what we expected
    innov = Z(:,k) - H * c_pred;            % m x 1
    S     = H * P_pred * H' + R;            % innovation covariance (m x m)

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
err_sq   = (c_hat - c_true).^2;                 % estimate error squared
resid    = Z - H * c_hat;                       % observed noise, m x N
resid_v  = resid(:);                            % all residuals pooled

fprintf('True constant           c     = %.4f\n', c_true);
fprintf('Final estimate          c_hat = %.4f\n', c_hat(end));
fprintf('Final covariance        P_N   = %.4e\n', P(end));
fprintf('Noise variance used     R     = %.4f\n', R_true);
fprintf('Sample variance of residuals  = %.4f   (should be close to R)\n', var(resid_v));
fprintf('Sample mean of residuals      = %.4f   (should be close to 0)\n', mean(resid_v));

% Sanity check: for a constant, the KF is exactly the running mean once
% P_0 is large. Compare.
running_mean = cumsum(mean(Z,1)) ./ (1:N);
fprintf('Max |KF estimate - running mean| = %.2e\n', max(abs(c_hat - running_mean)));

%% ---------------- Plots ----------------
k = 1:N;

% 1. Measured data
figure('Name','Measured data');
plot(k, Z', '.', 'Color', [0.5 0.5 0.5]); hold on;
plot([1 N], [c_true c_true], 'r-', 'LineWidth', 1.5);
xlabel('k'); ylabel('z_k'); grid on;
title(sprintf('Measured data  z_k = c + e_k,  e_k ~ N(0, %.2f)', R_true));
legend('measurements', 'true c', 'Location', 'best');

% 2. Recursive estimate vs true
figure('Name','Recursive estimate');
plot(k, Z', '.', 'Color', [0.75 0.75 0.75]); hold on;
plot(k, c_hat, 'b-', 'LineWidth', 1.5);
plot([1 N], [c_true c_true], 'r-', 'LineWidth', 1.5);
xlabel('k'); ylabel('c'); grid on;
title('Recursive Kalman estimate of the constant');
legend({'measurements', 'estimate $\hat{c}_k$', 'true $c$'}, 'Interpreter', 'latex', 'Location', 'best');

% 3. Error squared
figure('Name','Estimate error squared');
semilogy(k, err_sq, 'b-', 'LineWidth', 1.2); hold on;
semilogy(k, P, 'r--', 'LineWidth', 1.2);
xlabel('k'); grid on;
title('Estimate error squared vs. filter covariance');
legend({'$(\hat{c}_k - c)^2$', '$P_k$ (filter''s own error variance)'}, 'Interpreter', 'latex', 'Location', 'best');

% 4. Gain and covariance
figure('Name','Kalman gain and covariance');
subplot(2,1,1);
plot(k, Kg', 'b-', 'LineWidth', 1.2); grid on;
ylabel('K_k'); title('Kalman gain - decays like 1/k for a constant');
subplot(2,1,2);
semilogy(k, P, 'b-', 'LineWidth', 1.2); grid on;
xlabel('k'); ylabel('P_k'); title('Error covariance');

% 5. Observed Gaussian noise
figure('Name','Observed noise');
subplot(2,1,1);
plot(k, resid', '.', 'Color', [0.3 0.3 0.8]); hold on;
plot([1 N], [0 0], 'r-');
xlabel('k'); ylabel('$z_k - \hat{c}_k$', 'Interpreter', 'latex'); grid on;
title('Residuals (observed measurement noise)');
subplot(2,1,2);
histogram(resid_v, 30, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9]); hold on;
xx = linspace(min(resid_v), max(resid_v), 200);
gauss = exp(-xx.^2 / (2*sigma^2)) / (sigma*sqrt(2*pi));   % N(0, sigma^2) pdf
plot(xx, gauss, 'r-', 'LineWidth', 1.5);
xlabel('residual'); ylabel('pdf'); grid on;
title(sprintf('Zero-mean Gaussian, variance = %.3f  (R = %.3f)', var(resid_v), R_true));
legend('observed', 'N(0, R)', 'Location', 'best');

%% ---------------- Save every figure as PNG ----------------
save_all_figures(fullfile(fileparts(mfilename('fullpath')), 'figures'), 'partA');

%% ---------------- Exam-question experiments ----------------
% Uncomment one line at a time and re-run to see the effect.
%   R = 1e-6*eye(m);   % filter trusts measurements totally -> estimate = z_k
%   R = 1e6*eye(m);    % filter distrusts measurements -> estimate stays at c_prev
%   P_prev = 1e-6;     % filter is sure of its initial guess -> ignores data

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