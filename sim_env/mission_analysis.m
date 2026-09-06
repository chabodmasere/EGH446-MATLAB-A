function mission_analysis(opts)
% MISSION_ANALYSIS  Post-simulation controller / estimator analysis.
%
% Designed to be called from the model StopFcn so results appear
% immediately after the animation finishes:
%
%   set_param('sl_groundvehicleDynamics', 'StopFcn', 'mission_analysis();')
%
% Because this is a FUNCTION (own workspace), all simulation data is
% pulled from the base workspace with evalin(). Requires: logsout,
% wp_plot, wp_ordered, omega_max, v_max, capture_radius.
%
% All plots open in ONE window as tabs (uitabgroup):
%   1. Trajectory      - XY path vs planned route and waypoints
%   2. Heading         - cmd vs est heading + wrapped error
%   3. Commands        - v_cmd / omega_cmd with actuator limits
%   4. Estimator       - x / y / theta estimation errors
%   5. Waypoint Dist   - distance to active waypoint + capture radius
%   6. Cross-Track     - ground-truth finite-segment cross-track error
%
% Options:
%   mission_analysis(Save=true)   export each tab as a 300 DPI PNG

arguments
    opts.Save    (1,1) logical = false
    opts.SaveDir (1,:) char = fullfile(fileparts(mfilename('fullpath')), 'figures')
    opts.Prefix  (1,:) char = 'mission'
end

tag = 'mission_analysis';
close(findobj('Type', 'figure', 'Tag', tag));

%% ---------------- Pull simulation data from base workspace ----------------
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

%% ---------------- Extract logged signals ----------------
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

%% ---------------- Mission performance ----------------
mission_time = dist_to_wp_time(end);
route_length = sum(vecnorm(diff(wp_plot), 2, 2));

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

%% ---------------- Estimator accuracy ----------------
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

fprintf('\nMission\n-------\n');
fprintf('Mission Time                = %.3f s\n', mission_time);
fprintf('Planned Route Length        = %.3f m\n', route_length);
fprintf('Mission Time per Metre      = %.4f s/m\n', mission_time_per_metre);
fprintf('Effective Mission Speed     = %.3f m/s\n', mission_effective_speed);
fprintf('Waypoints Captured          = %d/%d\n', ...
    num_true_waypoints_captured, size(wp_ordered,1));

fprintf('\nHeading Tracking (controller, uses x_hat)\n');
fprintf('-----------------------------------------\n');
fprintf('RMS Heading Error           = %.3f deg\n', heading_error_rms);
fprintf('Mean Absolute Error         = %.3f deg\n', heading_error_mean_abs);
fprintf('Maximum Absolute Error      = %.3f deg\n', heading_error_max_abs);

fprintf('\nHeading Tracking (ground truth)\n');
fprintf('-------------------------------\n');
fprintf('TRUE RMS Heading Error      = %.3f deg\n', true_heading_error_rms);
fprintf('TRUE Mean Absolute Error    = %.3f deg\n', true_heading_error_mean_abs);
fprintf('TRUE Maximum Error          = %.3f deg\n', true_heading_error_max_abs);

fprintf('\nState Estimator Accuracy\n');
fprintf('------------------------\n');
fprintf('X Position RMSE             = %.3f m\n', estimator_rmse_x);
fprintf('Y Position RMSE             = %.3f m\n', estimator_rmse_y);
fprintf('2-D Position RMSE           = %.3f m\n', estimator_rmse_position);
fprintf('Heading Estimate RMSE       = %.3f deg\n', estimator_rmse_heading);

fprintf('\nController Commands\n-------------------\n');
fprintf('Maximum v_cmd               = %.3f m/s\n', max_v_cmd);
fprintf('Mean v_cmd                  = %.3f m/s\n', mean_v_cmd);
fprintf('Maximum |omega_cmd|         = %.3f rad/s\n', max_omega_cmd);
fprintf('Mean |omega_cmd|            = %.3f rad/s\n', mean_abs_omega_cmd);
fprintf('Yaw Saturation Time         = %.3f s\n', yaw_saturated_time);
fprintf('Yaw Saturation Percentage   = %.2f %%\n', yaw_saturation_percent);

fprintf('\nWaypoint Capture (estimated | true)\n');
fprintf('-----------------------------------\n');
fprintf('Average Capture Error       = %.3f m | %.3f m\n', ...
    mean_capture_error, mean_true_capture_error);
fprintf('Maximum Capture Error       = %.3f m | %.3f m\n', ...
    max_capture_error, max_true_capture_error);
fprintf('Minimum Capture Error       = %.3f m | %.3f m\n', ...
    min_capture_error, min_true_capture_error);

if num_true_waypoints_captured > 0
    fprintf('\nIndividual TRUE Capture Errors\n');
    fprintf('------------------------------\n');
    for i = 1:num_true_waypoints_captured
        fprintf('Waypoint %2d = %.3f m\n', ...
            true_capture_wp_indices(i), true_capture_errors(i));
    end
end

fprintf('\nCross-Track (ground truth, finite segment)\n');
fprintf('------------------------------------------\n');
fprintf('TRUE RMS Cross-Track Error  = %.3f m\n', rms_cross_track_error);
fprintf('TRUE Mean Cross-Track Error = %.3f m\n', mean_cross_track_error);
fprintf('TRUE Maximum CT Error       = %.3f m\n', max_cross_track_error);
fprintf('========================================\n');

%% ---------------- One window, tabbed plots ----------------
% uitabgroup puts a tab strip inside a single figure window; every set
% of axes is parented to a tab instead of the figure, so all plots live
% in one organised window.

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

%% Tab 1: XY trajectory vs waypoints
tab = uitab(tg, 'Title', 'Trajectory', 'BackgroundColor', 'w');
ax = axes(tab);
plot(ax, true_x, true_y, 'b-'); hold(ax, 'on');
plot(ax, x_hat_data(:,1), x_hat_data(:,2), 'g--');
plot(ax, wp_plot(:,1), wp_plot(:,2), 'k:', 'LineWidth', 1.2);
plot(ax, wp_ordered(:,1), wp_ordered(:,2), 'rp', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'r');
plot(ax, true_x(1), true_y(1), 'ks', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'k');
axis(ax, 'equal'); grid(ax, 'on');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
title(ax, 'Vehicle Trajectory vs Planned Route');
legend(ax, 'true path', 'estimated path', 'planned route', ...
    'waypoints', 'start', 'Location', 'best');

%% Tab 2: Heading tracking + wrapped error
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

%% Tab 3: Controller commands with actuator limits
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

%% Tab 4: Estimator errors
if ~isempty(t_est_valid)
    tab = uitab(tg, 'Title', 'Estimator', 'BackgroundColor', 'w');
    tl = tiledlayout(tab, 3, 1);

    ax = nexttile(tl);
    plot(ax, t_est_valid, x_est_error);
    grid(ax, 'on');
    ylabel(ax, 'X Error (m)');
    title(ax, sprintf('Kalman X-Position Error (RMSE = %.3f m)', ...
        estimator_rmse_x));

    ax = nexttile(tl);
    plot(ax, t_est_valid, y_est_error);
    grid(ax, 'on');
    ylabel(ax, 'Y Error (m)');
    title(ax, sprintf('Kalman Y-Position Error (RMSE = %.3f m)', ...
        estimator_rmse_y));

    ax = nexttile(tl);
    plot(ax, t_est_valid, rad2deg(theta_est_error));
    grid(ax, 'on');
    xlabel(ax, 'Time (s)'); ylabel(ax, 'Heading Error (deg)');
    title(ax, sprintf('Kalman Heading Error (RMSE = %.3f deg)', ...
        estimator_rmse_heading));
end

%% Tab 5: Distance to active waypoint
tab = uitab(tg, 'Title', 'Waypoint Dist', 'BackgroundColor', 'w');
ax = axes(tab);
plot(ax, dist_to_wp_time, dist_to_wp_data); hold(ax, 'on');
yline(ax, capture_radius, 'r--', 'capture radius', 'LineWidth', 1.6);
grid(ax, 'on');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Distance (m)');
title(ax, 'Estimated Distance to Active Waypoint');

%% Tab 6: Cross-track error
tab = uitab(tg, 'Title', 'Cross-Track', 'BackgroundColor', 'w');
ax = axes(tab);
plot(ax, pose_true_time(valid_ct), cross_track_valid);
grid(ax, 'on');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Distance to Active Path Segment (m)');
title(ax, sprintf( ...
    'Ground-Truth Cross-Track Error (RMS = %.3f m, max = %.3f m)', ...
    rms_cross_track_error, max_cross_track_error));

drawnow;

%% ---------------- Save ----------------
% exportgraphics is pointed at each TAB handle: exporting the figure
% would only capture whichever tab is currently visible.
if opts.Save
    save_tabs(tg, opts.SaveDir, opts.Prefix);
end

end

%% ---------------- Helper ----------------
function save_tabs(tabgroup, folder, prefix)
% Save every tab of the analysis window as a 300 DPI PNG.

    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    tabs = tabgroup.Children;

    for i = 1:numel(tabs)
        name = regexprep(lower(tabs(i).Title), '[^a-z0-9]+', '_');
        file = fullfile(folder, sprintf('%s_%02d_%s.png', prefix, i, name));

        exportgraphics(tabs(i), file, ...
            'Resolution', 300, 'BackgroundColor', 'white');
    end

    fprintf('Saved %d tabs to %s\n', numel(tabs), folder);
end
