function mission_analysis(src, opts)
% MISSION_ANALYSIS  Post-simulation GNC + LKF estimation analysis.
%
% Called from the model StopFcn so results appear right after the
% animation finishes:
%
%   set_param('sl_groundvehicleDynamics', 'StopFcn', 'mission_analysis();')
%
% IMPORTANT plots open as tabs in ONE window:
%   1. Trajectory   - measured / true / estimated path vs planned route
%   2. LKF vs Raw   - raw sensor error vs filtered error (why the LKF earns its place)
%   3. Est Error    - x / y / theta estimation errors with RMSE
%   4. Innovation   - innovation histograms vs N(0, var) (consistency check)
%   5. Heading      - cmd vs est heading + wrapped tracking error
%   6. Commands     - v_cmd / omega_cmd with actuator limits
%   7. Guidance     - distance to waypoint + cross-track error
%
% EVERYTHING (tabs + extra diagnostics) is saved as 300 DPI PNGs into an
% "images" folder next to this file. The extra diagnostics (states vs k,
% Kalman gains, covariance history, steady-state P heatmap) are rendered
% invisibly, saved, and closed - they never clutter the screen.
%
% Because this is a FUNCTION (own workspace), simulation data is pulled
% from the base workspace with evalin(). Requires: logsout, wp_plot,
% wp_ordered, omega_max, v_max, capture_radius, and the To Workspace
% logs pose_log, z_log, xhat_log, K_log, P_log, innov_log.
%
% Options:
%   mission_analysis(Save=false)      show tabs only, save nothing
%   mission_analysis(Verbose=true)    also dump the raw P, K and innovation
%                                     matrices after the summary

arguments
    src = []                       % optional Simulink.SimulationOutput
    opts.dt      (1,1) double  = 0.01
    opts.Save    (1,1) logical = true
    opts.SaveDir (1,:) char = fullfile(fileparts(mfilename('fullpath')), 'images')
    opts.Prefix  (1,:) char = 'mission'
    opts.Verbose (1,1) logical = false   % also dump raw P, K and innovation matrices
end

tag    = 'mission_analysis';
hidtag = 'mission_analysis_saved_only';
close(findobj('Type', 'figure', 'Tag', tag));
close(findobj('Type', 'figure', 'Tag', hidtag));

%% ---------------- Pull GNC data from base workspace ----------------
try
    logsout = evalin('base', 'logsout');
catch
    error('mission_analysis:noLogsout', ...
        'logsout not found in base workspace. Run the simulation first.');
end

wp_plot        = evalin('base', 'wp_plot');
wp_ordered     = evalin('base', 'wp_ordered');
omega_max      = evalin('base', 'omega_max');
v_max          = evalin('base', 'v_max');
capture_radius = evalin('base', 'capture_radius');

%% ---------------- Extract logged GNC signals ----------------
getsig = @(name) logsout.getElement(name).Values;

theta_cmd     = getsig('theta_cmd');
theta_hat     = getsig('theta_hat');
e_wrap        = getsig('e_wrap');
omega_cmd     = getsig('omega_cmd');
v_cmd         = getsig('v_cmd');
dist_to_wp    = getsig('dist_to_wp');
pose_true     = getsig('pose_true');
x_hat         = getsig('x_hat');
wp_idx_out    = getsig('wp_idx_out');
capture_event = getsig('capture_event');
capture_error = getsig('capture_error');

col = @(ts) deal(ts.Time(:), reshape(squeeze(ts.Data), [], 1));

[theta_cmd_time,     theta_cmd_data]     = col(theta_cmd);
[theta_hat_time,     theta_hat_data]     = col(theta_hat);
[e_wrap_time,        e_wrap_data]        = col(e_wrap);
[omega_cmd_time,     omega_cmd_data]     = col(omega_cmd);
[v_cmd_time,         v_cmd_data]         = col(v_cmd);
[dist_to_wp_time,    dist_to_wp_data]    = col(dist_to_wp);
[capture_event_time, capture_event_data] = col(capture_event);
[capture_error_time, capture_error_data] = col(capture_error);
[wp_idx_time,        wp_idx_data]        = col(wp_idx_out);

% Ground-truth pose [x, y, theta] -> rows = time samples
pose_true_time = pose_true.Time(:);
pose_true_data = squeeze(pose_true.Data);
if size(pose_true_data,1) ~= length(pose_true_time) && ...
        size(pose_true_data,2) == length(pose_true_time)
    pose_true_data = pose_true_data.';
end
if size(pose_true_data,2) < 3
    error('pose_true does not contain enough states. Expected [x, y, theta].');
end
true_x     = pose_true_data(:,1);
true_y     = pose_true_data(:,2);
true_theta = pose_true_data(:,3);

% Estimator state [x, y, theta, ...] -> rows = time samples
x_hat_time = x_hat.Time(:);
x_hat_data = squeeze(x_hat.Data);
if size(x_hat_data,1) ~= length(x_hat_time) && ...
        size(x_hat_data,2) == length(x_hat_time)
    x_hat_data = x_hat_data.';
end
if size(x_hat_data,2) < 3
    error('x_hat does not contain enough states. Expected at least [x, y, theta].');
end

%% ---------------- Load LKF diagnostic logs (To Workspace) ----------------
% pose_log / z_log / xhat_log / K_log / P_log / innov_log. If they are
% missing, the GNC analysis still runs; the estimation tabs are skipped.
haveLKF = true;
try
    names = {'pose_log','z_log','xhat_log','K_log','P_log','innov_log'};
    S = load_signals(src, names);

    [xhatL,  t_x] = grab(S.xhat_log,  opts.dt);
    [poseL,  t_p] = grab(S.pose_log,  opts.dt);
    [zL,     t_z] = grab(S.z_log,     opts.dt);
    [innovL, t_i] = grab(S.innov_log, opts.dt);
    [KL,     t_K] = grab3(S.K_log,    opts.dt);
    [PL,     t_P] = grab3(S.P_log,    opts.dt);

    tL     = t_x(:);
    poseL  = align(poseL,  t_p, tL);
    zL     = align(zL,     t_z, tL);
    innovL = align(innovL, t_i, tL);
    KL     = align3(KL, t_K, tL);
    PL     = align3(PL, t_P, tL);

    NL = numel(tL);
    kL = (1:NL)';

    wrap = @(a) atan2(sin(a), cos(a));

    errL           = poseL - xhatL(:,1:size(poseL,2));
    errL(:,3)      = wrap(errL(:,3));
    meas_errL      = poseL - zL(:,1:3);
    meas_errL(:,3) = wrap(meas_errL(:,3));

    rmse_est  = sqrt(mean(errL.^2));
    rmse_meas = sqrt(mean(meas_errL.^2));
catch ME
    haveLKF = false;
    warning('mission_analysis:noLKFLogs', ...
        'LKF logs not found (%s). Skipping estimation diagnostics.', ...
        ME.message);
end

%% ---------------- Mission performance ----------------
mission_time = dist_to_wp_time(end);
route_length = sum(vecnorm(diff(wp_plot), 2, 2));

% Distance the vehicle actually drove, from the true pose history.
if exist('poseL', 'var') && ~isempty(poseL) && size(poseL, 1) > 1
    distance_travelled = sum(vecnorm(diff(poseL(:,1:2)), 2, 2));
    tracking_overhead  = distance_travelled - route_length;
else
    distance_travelled = NaN;
    tracking_overhead  = NaN;
end

if route_length > eps
    mission_time_per_metre = mission_time / route_length;
    mission_effective_speed = route_length / mission_time;
else
    mission_time_per_metre = NaN;
    mission_effective_speed = NaN;
end

%% ---------------- Heading tracking ----------------
% unwrap() only for display: removes artificial +/-180 deg jumps.
t_heading = theta_hat_time;
e_interp = interp1(e_wrap_time, e_wrap_data, t_heading, 'linear', 'extrap');
e_interp = e_interp(:);

theta_hat_unwrapped = unwrap(theta_hat_data);
theta_cmd_continuous = theta_hat_unwrapped + e_interp;

heading_error_deg = rad2deg(e_wrap_data);
heading_error_rms = sqrt(mean(heading_error_deg.^2));
heading_error_mean_abs = mean(abs(heading_error_deg));
heading_error_max_abs = max(abs(heading_error_deg));

% Ground-truth heading error (cmd vs TRUE heading)
theta_cmd_on_true = interp1(theta_cmd_time, theta_cmd_data, ...
    pose_true_time, 'linear', 'extrap');
theta_cmd_on_true = theta_cmd_on_true(:);

true_heading_error = atan2( ...
    sin(theta_cmd_on_true - true_theta), ...
    cos(theta_cmd_on_true - true_theta));
true_heading_error_deg = rad2deg(true_heading_error);

true_heading_error_rms      = sqrt(mean(true_heading_error_deg.^2));
true_heading_error_mean_abs = mean(abs(true_heading_error_deg));
true_heading_error_max_abs  = max(abs(true_heading_error_deg));

%% ---------------- Estimator accuracy (x_hat vs ground truth) ----------------
valid_est_time = pose_true_time >= x_hat_time(1) & ...
                 pose_true_time <= x_hat_time(end);

t_est_compare      = pose_true_time(valid_est_time);
true_x_compare     = true_x(valid_est_time);
true_y_compare     = true_y(valid_est_time);
true_theta_compare = true_theta(valid_est_time);

x_hat_interp     = interp1(x_hat_time, x_hat_data(:,1), t_est_compare, 'linear');
y_hat_interp     = interp1(x_hat_time, x_hat_data(:,2), t_est_compare, 'linear');
theta_hat_interp = interp1(x_hat_time, x_hat_data(:,3), t_est_compare, 'linear');

x_est_error = x_hat_interp(:) - true_x_compare;
y_est_error = y_hat_interp(:) - true_y_compare;
theta_est_error = atan2( ...
    sin(theta_hat_interp(:) - true_theta_compare), ...
    cos(theta_hat_interp(:) - true_theta_compare));

valid_estimator = isfinite(x_est_error) & ...
                  isfinite(y_est_error) & ...
                  isfinite(theta_est_error);

x_est_error     = x_est_error(valid_estimator);
y_est_error     = y_est_error(valid_estimator);
theta_est_error = theta_est_error(valid_estimator);
t_est_valid     = t_est_compare(valid_estimator);

if ~isempty(x_est_error)
    estimator_rmse_x = sqrt(mean(x_est_error.^2));
    estimator_rmse_y = sqrt(mean(y_est_error.^2));
    estimator_rmse_position = sqrt(mean(x_est_error.^2 + y_est_error.^2));
    estimator_rmse_heading = rad2deg(sqrt(mean(theta_est_error.^2)));
else
    [estimator_rmse_x, estimator_rmse_y, ...
     estimator_rmse_position, estimator_rmse_heading] = deal(NaN);
end

%% ---------------- Controller command statistics ----------------
max_v_cmd = max(v_cmd_data);
mean_v_cmd = mean(v_cmd_data);
max_omega_cmd = max(abs(omega_cmd_data));
mean_abs_omega_cmd = mean(abs(omega_cmd_data));

% Yaw-rate saturation: |omega_cmd| >= 99% of omega_max
yaw_is_saturated = abs(omega_cmd_data) >= 0.99 * omega_max;

if length(omega_cmd_time) > 1
    yaw_saturated_time = trapz(omega_cmd_time, double(yaw_is_saturated));
    yaw_total_time = omega_cmd_time(end) - omega_cmd_time(1);
    if yaw_total_time > 0
        yaw_saturation_percent = 100 * yaw_saturated_time / yaw_total_time;
    else
        yaw_saturation_percent = NaN;
    end
else
    yaw_saturated_time = NaN;
    yaw_saturation_percent = NaN;
end

%% ---------------- Estimated waypoint capture error ----------------
captured = capture_event_data > 0.5;

if length(capture_error_data) ~= length(capture_event_data) || ...
        any(capture_error_time ~= capture_event_time)
    capture_error_aligned = interp1(capture_error_time, capture_error_data, ...
        capture_event_time, 'previous', 'extrap');
else
    capture_error_aligned = capture_error_data;
end
capture_error_aligned = capture_error_aligned(:);

waypoint_capture_errors = capture_error_aligned(captured);
waypoint_capture_errors = waypoint_capture_errors( ...
    isfinite(waypoint_capture_errors) & waypoint_capture_errors >= 0);

num_waypoints_captured = length(waypoint_capture_errors);

if num_waypoints_captured > 0
    mean_capture_error = mean(waypoint_capture_errors);
    max_capture_error = max(waypoint_capture_errors);
    min_capture_error = min(waypoint_capture_errors);
else
    [mean_capture_error, max_capture_error, min_capture_error] = deal(NaN);
end

%% ---------------- Ground-truth waypoint capture error ----------------
% Guidance decides capture with x_hat; here we measure the TRUE physical
% miss distance at the instant capture is declared.
capture_samples = find(captured);
true_capture_errors = [];
true_capture_wp_indices = [];

for k = 1:length(capture_samples)
    t_cap = capture_event_time(capture_samples(k));

    x_true_cap = interp1(pose_true_time, true_x, t_cap, 'linear', 'extrap');
    y_true_cap = interp1(pose_true_time, true_y, t_cap, 'linear', 'extrap');

    wp_cap = round(interp1(wp_idx_time, double(wp_idx_data), ...
        t_cap, 'previous', 'extrap'));

    if wp_cap >= 1 && wp_cap <= size(wp_ordered,1)
        wp_x = wp_ordered(wp_cap,1);
        wp_y = wp_ordered(wp_cap,2);
        true_capture_errors(end+1,1) = ...
            sqrt((x_true_cap - wp_x)^2 + (y_true_cap - wp_y)^2); %#ok<AGROW>
        true_capture_wp_indices(end+1,1) = wp_cap; %#ok<AGROW>
    end
end

valid_true_capture = isfinite(true_capture_errors) & true_capture_errors >= 0;
true_capture_errors = true_capture_errors(valid_true_capture);
true_capture_wp_indices = true_capture_wp_indices(valid_true_capture);

num_true_waypoints_captured = length(true_capture_errors);

if num_true_waypoints_captured > 0
    mean_true_capture_error = mean(true_capture_errors);
    max_true_capture_error = max(true_capture_errors);
    min_true_capture_error = min(true_capture_errors);
else
    [mean_true_capture_error, max_true_capture_error, ...
     min_true_capture_error] = deal(NaN);
end

%% ---------------- Ground-truth finite-segment cross-track error ----------------
% While targeting waypoint i, the desired segment is
% wp_plot(i,:) -> wp_plot(i+1,:). Distance from TRUE position to the
% closest point on the ACTIVE finite segment.
wp_index = interp1(wp_idx_time, double(wp_idx_data), ...
    pose_true_time, 'previous', 'extrap');
wp_index = round(wp_index(:));

route = wp_plot;
cross_track_error = NaN(length(pose_true_time),1);

for k = 1:length(pose_true_time)
    i = wp_index(k);
    if i >= 1 && i < size(route,1)
        A = route(i,:);
        B = route(i+1,:);
        P = [true_x(k), true_y(k)];

        segment = B - A;
        seg_len_sq = dot(segment, segment);

        if seg_len_sq > eps
            t_proj = dot(P - A, segment) / seg_len_sq;
            t_clamped = max(0, min(1, t_proj));
            closest_point = A + t_clamped * segment;
            cross_track_error(k) = norm(P - closest_point);
        end
    end
end

valid_ct = isfinite(cross_track_error);
cross_track_valid = cross_track_error(valid_ct);

if ~isempty(cross_track_valid)
    rms_cross_track_error = sqrt(mean(cross_track_valid.^2));
    mean_cross_track_error = mean(cross_track_valid);
    max_cross_track_error = max(cross_track_valid);
else
    [rms_cross_track_error, mean_cross_track_error, ...
     max_cross_track_error] = deal(NaN);
end

%% ---------------- Console summary ----------------
fprintf('\n========================================\n');
fprintf('EGH446 Mission Performance Summary\n');
fprintf('========================================\n');

fprintf('\nMISSION\n-------\n');
fprintf('Mission time                = %.2f s\n', mission_time);
fprintf('Planned route length        = %.2f m\n', route_length);
fprintf('Distance travelled          = %.2f m\n', distance_travelled);
fprintf('Tracking overhead           = %.2f m (%.1f%% over plan)\n', ...
    tracking_overhead, 100*tracking_overhead/route_length);
fprintf('Average speed               = %.2f m/s\n', mission_effective_speed);
fprintf('Waypoints captured          = %d/%d\n', ...
    num_waypoints_captured, size(wp_plot,1)-1);

fprintf('\nHOW WELL IT FOLLOWED THE ROUTE\n');
fprintf('------------------------------\n');
fprintf('Mean waypoint capture error = %.3f m\n', mean_true_capture_error);
fprintf('Max  waypoint capture error = %.3f m\n', max_true_capture_error);
fprintf('RMS cross-track error       = %.3f m\n', rms_cross_track_error);
fprintf('Max cross-track error       = %.3f m\n', max_cross_track_error);
fprintf('RMS heading error           = %.2f deg\n', true_heading_error_rms);
fprintf('Yaw rate saturated          = %.1f%% of the mission\n', ...
    yaw_saturation_percent);

if haveLKF
    fprintf('\nHOW WELL IT KNEW WHERE IT WAS\n');
    fprintf('-----------------------------\n');
    fprintf('Position RMSE, filtered     = %.3f m  (raw %.3f m)\n', ...
        estimator_rmse_position, hypot(rmse_meas(1), rmse_meas(2)));
    fprintf('Heading RMSE, filtered      = %.2f deg\n', estimator_rmse_heading);
    fprintf('Filter improvement          = %.1fx in x, %.1fx in y, %.1fx in theta\n', ...
        rmse_meas(1)/rmse_est(1), rmse_meas(2)/rmse_est(2), ...
        rmse_meas(3)/rmse_est(3));
end

fprintf('\nCOMMANDS\n--------\n');
fprintf('Speed, mean / max           = %.2f / %.2f m/s\n', ...
    mean_v_cmd, max_v_cmd);
fprintf('Turn rate, mean / max       = %.2f / %.2f rad/s\n', ...
    mean_abs_omega_cmd, max_omega_cmd);

%% ---------------- Detail, only on request ----------------
if opts.Verbose

    fprintf('\nHeading tracking, controller view (uses x_hat)\n');
    fprintf('  RMS %.3f deg | mean abs %.3f deg | max %.3f deg\n', ...
        heading_error_rms, heading_error_mean_abs, heading_error_max_abs);
    fprintf('Heading tracking, ground truth\n');
    fprintf('  RMS %.3f deg | mean abs %.3f deg | max %.3f deg\n', ...
        true_heading_error_rms, true_heading_error_mean_abs, ...
        true_heading_error_max_abs);

    fprintf('Estimator RMSE: x %.3f m | y %.3f m | 2-D %.3f m | theta %.3f deg\n', ...
        estimator_rmse_x, estimator_rmse_y, estimator_rmse_position, ...
        estimator_rmse_heading);

    fprintf('Capture error, estimated vs true\n');
    fprintf('  mean %.3f | %.3f m\n', mean_capture_error, mean_true_capture_error);
    fprintf('  max  %.3f | %.3f m\n', max_capture_error,  max_true_capture_error);
    fprintf('  min  %.3f | %.3f m\n', min_capture_error,  min_true_capture_error);

    if haveLKF
        fprintf('\nSteady-state covariance P (last step):\n'); disp(PL(:,:,end));
        fprintf('Steady-state Kalman gain K (last step):\n');  disp(KL(:,:,end));
        fprintf('Innovation variance (should be near R + H P H''):\n');
        disp(var(innovL, 0, 1));
    end
end

fprintf('========================================\n');

%% ---------------- One window, tabbed plots (IMPORTANT ones) ----------------
fig = figure( ...
    'Name', 'EGH446 Mission Analysis', ...
    'Tag', tag, ...
    'Color', 'w', ...
    'NumberTitle', 'off', ...
    'Units', 'pixels', ...
    'Position', [80 80 1150 720], ...
    'DefaultAxesFontName', 'Arial', ...
    'DefaultAxesFontSize', 13, ...
    'DefaultAxesFontWeight', 'bold', ...
    'DefaultAxesLineWidth', 1.6, ...
    'DefaultAxesTitleFontWeight', 'bold', ...
    'DefaultLegendFontWeight', 'bold', ...
    'DefaultLineLineWidth', 2.2);

tg = uitabgroup(fig);

%% Tab 1: XY trajectory (measured / true / estimated / planned route)
tab = uitab(tg, 'Title', 'Trajectory', 'BackgroundColor', 'w');
ax = axes(tab);
if haveLKF
    plot(ax, zL(:,1), zL(:,2), '.', ...
        'Color', [0.75 0.75 0.75], 'MarkerSize', 4);
    hold(ax, 'on');
end
plot(ax, true_x, true_y, 'r-'); hold(ax, 'on');
plot(ax, x_hat_data(:,1), x_hat_data(:,2), 'b--');
plot(ax, wp_plot(:,1), wp_plot(:,2), 'k:', 'LineWidth', 1.2);
plot(ax, wp_ordered(:,1), wp_ordered(:,2), 'rp', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'r');
plot(ax, true_x(1), true_y(1), 'ks', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'k');
axis(ax, 'equal'); grid(ax, 'on');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
title(ax, 'Vehicle Trajectory vs Planned Route');
if haveLKF
    legend(ax, 'measured', 'true path', 'estimated path', ...
        'planned route', 'waypoints', 'start', 'Location', 'best');
else
    legend(ax, 'true path', 'estimated path', 'planned route', ...
        'waypoints', 'start', 'Location', 'best');
end

%% Tab 2: LKF vs raw sensor (why the filter earns its place)
if haveLKF
    tab = uitab(tg, 'Title', 'LKF vs Raw', 'BackgroundColor', 'w');
    tl = tiledlayout(tab, 2, 1);

    ax = nexttile(tl);
    plot(ax, kL, meas_errL(:,1), '-', 'Color', [0.7 0.7 0.7], ...
        'LineWidth', 1.2);
    hold(ax, 'on');
    plot(ax, kL, errL(:,1), 'b-');
    grid(ax, 'on');
    ylabel(ax, 'x error (m)');
    title(ax, sprintf( ...
        'Raw measurement error (grey) vs LKF error (blue), %.2fx better', ...
        rmse_meas(1)/rmse_est(1)));

    ax = nexttile(tl);
    plot(ax, kL, meas_errL(:,3), '-', 'Color', [0.7 0.7 0.7], ...
        'LineWidth', 1.2);
    hold(ax, 'on');
    plot(ax, kL, errL(:,3), 'b-');
    grid(ax, 'on');
    xlabel(ax, 'k'); ylabel(ax, '\theta error (rad)');
    title(ax, sprintf('Heading: raw vs LKF, %.2fx better', ...
        rmse_meas(3)/rmse_est(3)));
end

%% Tab 3: Estimation errors
if ~isempty(t_est_valid)
    tab = uitab(tg, 'Title', 'Est Error', 'BackgroundColor', 'w');
    tl = tiledlayout(tab, 3, 1);

    ax = nexttile(tl);
    plot(ax, t_est_valid, x_est_error);
    grid(ax, 'on');
    ylabel(ax, 'X Error (m)');
    title(ax, sprintf('LKF X-Position Error (RMSE = %.3f m)', ...
        estimator_rmse_x));

    ax = nexttile(tl);
    plot(ax, t_est_valid, y_est_error);
    grid(ax, 'on');
    ylabel(ax, 'Y Error (m)');
    title(ax, sprintf('LKF Y-Position Error (RMSE = %.3f m)', ...
        estimator_rmse_y));

    ax = nexttile(tl);
    plot(ax, t_est_valid, rad2deg(theta_est_error));
    grid(ax, 'on');
    xlabel(ax, 'Time (s)'); ylabel(ax, 'Heading Error (deg)');
    title(ax, sprintf('LKF Heading Error (RMSE = %.3f deg)', ...
        estimator_rmse_heading));
end

%% Tab 4: Innovation histograms (consistency evidence)
if haveLKF
    tab = uitab(tg, 'Title', 'Innovation', 'BackgroundColor', 'w');
    tl = tiledlayout(tab, 2, 2);
    inames = {'x', 'y', '\theta', 'v'};
    nInn = min(4, size(innovL, 2));

    for i = 1:nInn
        ax = nexttile(tl);
        histogram(ax, innovL(:,i), 40, 'Normalization', 'pdf', ...
            'FaceColor', [0.6 0.7 0.9]);
        hold(ax, 'on');
        s  = std(innovL(:,i));
        xx = linspace(min(innovL(:,i)), max(innovL(:,i)), 200);
        plot(ax, xx, exp(-xx.^2/(2*s^2))/(s*sqrt(2*pi)), 'r-', ...
            'LineWidth', 1.8);
        grid(ax, 'on');
        title(ax, sprintf('innovation %s: var = %.4f', inames{i}, s^2));
    end
    title(tl, 'Innovations z_k - Hx_k should be zero-mean Gaussian', ...
        'FontWeight', 'bold');
end

%% Tab 5: Heading tracking + wrapped error
tab = uitab(tg, 'Title', 'Heading', 'BackgroundColor', 'w');
tl = tiledlayout(tab, 2, 1);

ax = nexttile(tl);
plot(ax, t_heading, rad2deg(theta_cmd_continuous)); hold(ax, 'on');
plot(ax, t_heading, rad2deg(theta_hat_unwrapped));
grid(ax, 'on');
ylabel(ax, 'Heading (deg)');
title(ax, 'Desired and Estimated Vehicle Heading');
legend(ax, '\theta_{cmd}', '\theta_{hat}', 'Location', 'best');

ax = nexttile(tl);
plot(ax, e_wrap_time, rad2deg(e_wrap_data));
grid(ax, 'on');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Error (deg)');
title(ax, sprintf('Wrapped Heading Tracking Error (RMS = %.2f deg)', ...
    heading_error_rms));

%% Tab 6: Controller commands with actuator limits
tab = uitab(tg, 'Title', 'Commands', 'BackgroundColor', 'w');
tl = tiledlayout(tab, 2, 1);

ax = nexttile(tl);
plot(ax, v_cmd_time, v_cmd_data); hold(ax, 'on');
yline(ax, v_max, 'r--', 'v_{max}', 'LineWidth', 1.6);
grid(ax, 'on');
ylabel(ax, 'v_{cmd} (m/s)');
title(ax, 'Linear Velocity Command');

ax = nexttile(tl);
plot(ax, omega_cmd_time, omega_cmd_data); hold(ax, 'on');
yline(ax, omega_max, 'r--', '\omega_{max}', 'LineWidth', 1.6);
yline(ax, -omega_max, 'r--', 'LineWidth', 1.6);
grid(ax, 'on');
xlabel(ax, 'Time (s)'); ylabel(ax, '\omega_{cmd} (rad/s)');
title(ax, sprintf('Yaw Rate Command (saturated %.1f%% of mission)', ...
    yaw_saturation_percent));

%% Tab 7: Guidance (distance to waypoint + cross-track)
tab = uitab(tg, 'Title', 'Guidance', 'BackgroundColor', 'w');
tl = tiledlayout(tab, 2, 1);

ax = nexttile(tl);
plot(ax, dist_to_wp_time, dist_to_wp_data); hold(ax, 'on');
yline(ax, capture_radius, 'r--', 'capture radius', 'LineWidth', 1.6);
grid(ax, 'on');
ylabel(ax, 'Distance (m)');
title(ax, 'Estimated Distance to Active Waypoint');

ax = nexttile(tl);
plot(ax, pose_true_time(valid_ct), cross_track_valid);
grid(ax, 'on');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Cross-Track (m)');
title(ax, sprintf( ...
    'Ground-Truth Cross-Track Error (RMS = %.3f m, max = %.3f m)', ...
    rms_cross_track_error, max_cross_track_error));

drawnow;

%% ---------------- Saved-only diagnostics (invisible -> images/) ----------------
% These are report material, not every-run material: rendered with
% Visible off, exported, then closed. They never appear on screen.
if opts.Save && haveLKF

    kz = min(300, NL);
    nS = size(PL, 1);
    slabels = {'x','y','\theta','v','\omega','a_x','a_y'};
    slabels = slabels(1:min(nS, numel(slabels)));

    hidfig = @(name) figure('Name', name, 'Tag', hidtag, ...
        'Visible', 'off', 'Color', 'w', ...
        'Units', 'pixels', 'Position', [80 80 1150 720], ...
        'DefaultAxesFontName', 'Arial', ...
        'DefaultAxesFontSize', 12, ...
        'DefaultAxesFontWeight', 'bold', ...
        'DefaultAxesLineWidth', 1.4, ...
        'DefaultLineLineWidth', 1.6);

    % A. State estimates vs k
    hidfig('State estimates vs k');
    wrap = @(a) atan2(sin(a), cos(a));
    pose_p = poseL; z_p = zL;
    pose_p(:,3) = wrap(pose_p(:,3)); z_p(:,3) = wrap(z_p(:,3));
    labels = {'x (m)', 'y (m)', '\theta (rad, wrapped)'};
    for i = 1:3
        subplot(3,1,i);
        plot(kL, z_p(:,i), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3);
        hold on;
        plot(kL, pose_p(:,i), 'r-', 'LineWidth', 1.4);
        plot(kL, xhatL(:,i), 'b-', 'LineWidth', 1.0);
        ylabel(labels{i}); grid on;
        if i == 1
            title('Measured data, true state (red) and LKF estimate (blue)');
        end
        if i == 3, xlabel('k'); end
    end
    legend('measured', 'true', 'estimate', 'Location', 'best');

    % B. Kalman gains
    hidfig('Kalman gain');
    nG = min(size(KL,1), size(KL,2));
    subplot(2,1,1);
    hold on;
    for i = 1:nG
        plot(kL, squeeze(KL(i,i,:)));
    end
    grid on; ylabel('gain');
    title('Diagonal Kalman gains, full run: converge to steady state');
    legend(slabels(1:nG), 'Location', 'best');
    subplot(2,1,2);
    hold on;
    for i = 1:nG
        plot(kL(1:kz), squeeze(KL(i,i,1:kz)));
    end
    grid on; xlabel('k'); ylabel('gain');
    title(sprintf('Same gains, first %d steps: large while P is large', kz));

    % C. Covariance history
    hidfig('Error covariance');
    Pd = zeros(NL, nS);
    for i = 1:nS, Pd(:,i) = squeeze(PL(i,i,:)); end
    subplot(2,1,1);
    semilogy(kL, Pd); grid on; ylabel('P_{ii}');
    title('Diagonal of P, full run: filter''s own variance per state');
    legend(slabels, 'Location', 'best');
    subplot(2,1,2);
    semilogy(kL(1:kz), Pd(1:kz,:)); grid on;
    xlabel('k'); ylabel('P_{ii}');
    title(sprintf('First %d steps: uncertainty collapses to steady state', kz));

    % D. Steady-state covariance heatmap
    hidfig('Steady-state covariance');
    imagesc(PL(:,:,end)); colorbar; axis square;
    set(gca, 'XTick', 1:nS, 'YTick', 1:nS, ...
        'XTickLabel', slabels, 'YTickLabel', slabels);
    title('Full-state covariance matrix P at final step');
end

%% ---------------- Save everything to the images folder ----------------
if opts.Save

    if ~exist(opts.SaveDir, 'dir')
        mkdir(opts.SaveDir);
    end

    idx = 0;

    % Tabs: exportgraphics must be pointed at each TAB handle, because
    % exporting the figure would only capture the visible tab.
    % tg.Children is already in left-to-right tab order, so do not reverse
    % it; flipping here numbers the saved images backwards.
    tabs = tg.Children(:);
    for i = 1:numel(tabs)
        idx = idx + 1;
        export_one(tabs(i), tabs(i).Title, opts.SaveDir, opts.Prefix, idx);
    end

    % Saved-only diagnostics, then close them.
    hid = findobj('Type', 'figure', 'Tag', hidtag);
    if ~isempty(hid)
        [~, order] = sort([hid.Number]);
        hid = hid(order);
        for i = 1:numel(hid)
            idx = idx + 1;
            export_one(hid(i), hid(i).Name, opts.SaveDir, opts.Prefix, idx);
        end
        close(hid);
    end

    fprintf('\nSaved %d images to %s\n', idx, opts.SaveDir);
    fprintf('  %d from the tabs on screen, %d report-only diagnostics.\n', ...
        numel(tabs), idx - numel(tabs));
end

end

%% =====================================================================
%% helpers
%% =====================================================================
function export_one(h, name, folder, prefix, idx)
% Export one tab or figure handle as a 300 DPI PNG.
    name = regexprep(lower(name), '[^a-z0-9]+', '_');
    file = fullfile(folder, sprintf('%s_%02d_%s.png', prefix, idx, name));
    exportgraphics(h, file, 'Resolution', 300, 'BackgroundColor', 'white');
end

function S = load_signals(src, names)
% Collect logged variables into struct S. src may be a
% Simulink.SimulationOutput, a struct with those fields, or [].
% If []: look in the base workspace for the loose variables, or for any
% SimulationOutput object holding them.
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
                    return;
                catch
                end
            end
        end
        error('No LKF logs (xhat_log etc.) found in base workspace.');
    end

    for i = 1:numel(names)
        if isa(src, 'Simulink.SimulationOutput')
            if ~any(strcmp(src.who, names{i}))
                error('Sim output has no ''%s''.', names{i});
            end
            S.(names{i}) = src.get(names{i});
        elseif isstruct(src) && isfield(src, names{i})
            S.(names{i}) = src.(names{i});
        else
            error('Input has no ''%s''.', names{i});
        end
    end
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