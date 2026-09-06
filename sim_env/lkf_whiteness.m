function lkf_whiteness(opts)
% LKF_WHITENESS  Innovation autocorrelation: the direct test of filter
% consistency. White innovations = the filter extracts all information.
% Correlated innovations = something repeats or lags: held noise, shared
% seeds, or unmodelled dynamics. State errors exceed P in exactly that case.

arguments
    opts.k0    (1,1) double = 500     % same steady-state window as metrics
    opts.nlags (1,1) double = 50
end

innov = evalin('base', 'innov_log');
if isa(innov, 'timeseries'), innov = squeeze(innov.Data); end
if size(innov,1) < size(innov,2), innov = innov'; end

v  = innov(opts.k0:end, :);           % steady-state window
N  = size(v,1);
ci = 2/sqrt(N);                       % 95% bound for white noise
ch = {'x','y','theta','v'};

fprintf('Innovation autocorrelation, lags 1..5 (95%% white-noise bound: +/-%.4f)\n', ci);
fprintf('lag        x         y      theta        v\n');
rho = zeros(opts.nlags, 4);
for c = 1:4
    s = v(:,c) - mean(v(:,c));
    den = sum(s.^2);
    for L = 1:opts.nlags
        rho(L,c) = sum(s(1+L:end).*s(1:end-L)) / den;
    end
end
for L = 1:5
    fprintf('%3d  %9.4f %9.4f %9.4f %9.4f\n', L, rho(L,:));
end

% Cross-correlation at lag 0 between channels: nonzero => shared noise
% seeds or common-mode corruption. Should be ~0 (same +/-2/sqrt(N) bound).
C = corrcoef(v);
fprintf('\nCross-correlation at lag 0 (off-diagonals should be ~0):\n');
disp(C);

figure('Name','Innovation whiteness','Color','w');
for c = 1:4
    subplot(2,2,c);
    stem(1:opts.nlags, rho(:,c), 'filled', 'MarkerSize', 3); hold on;
    yline( ci, 'r--'); yline(-ci, 'r--'); grid on;
    title(sprintf('\\rho_{%s}(L)   lag-1 = %.3f', ch{c}, rho(1,c)));
    xlabel('lag L'); ylim([-0.2 1]);
end
end