%% EGH446 Controller Analysis
% Analyse logged controller, guidance, estimator and vehicle signals
% after Simulink simulation.
%
% The controller operates using the Kalman-filtered state estimate x_hat.
% Ground-truth vehicle pose is used ONLY for offline performance
% evaluation and verification.

close all;


%% Clear Analysis Variables From Previous Runs

clear theta_cmd_continuous theta_hat_unwrapped e_interp
clear heading_error_deg true_heading_error_deg
clear cross_track_error cross_track_valid
clear true_capture_errors
clear x_est_error y_est_error theta_est_error
clear estimator_rmse_x estimator_rmse_y
clear estimator_rmse_position estimator_rmse_heading
clear yaw_is_saturated yaw_saturation_percent yaw_saturated_time


%% Extract Logged Signals

theta_cmd  = logsout.getElement('theta_cmd').Values;
theta_hat  = logsout.getElement('theta_hat').Values;
e_wrap     = logsout.getElement('e_wrap').Values;
omega_cmd  = logsout.getElement('omega_cmd').Values;
v_cmd      = logsout.getElement('v_cmd').Values;
dist_to_wp = logsout.getElement('dist_to_wp').Values;

% Ground-truth simulator pose
pose_true = logsout.getElement('pose_true').Values;

% Additional navigation-analysis signals
x_hat         = logsout.getElement('x_hat').Values;
wp_idx_out    = logsout.getElement('wp_idx_out').Values;
capture_event = logsout.getElement('capture_event').Values;
capture_error = logsout.getElement('capture_error').Values;


%% Convert Scalar Logged Signals to Column Vectors

theta_cmd_time = theta_cmd.Time(:);
theta_cmd_data = squeeze(theta_cmd.Data);
theta_cmd_data = theta_cmd_data(:);

theta_hat_time = theta_hat.Time(:);
theta_hat_data = squeeze(theta_hat.Data);
theta_hat_data = theta_hat_data(:);

e_wrap_time = e_wrap.Time(:);
e_wrap_data = squeeze(e_wrap.Data);
e_wrap_data = e_wrap_data(:);

omega_cmd_time = omega_cmd.Time(:);
omega_cmd_data = squeeze(omega_cmd.Data);
omega_cmd_data = omega_cmd_data(:);

v_cmd_time = v_cmd.Time(:);
v_cmd_data = squeeze(v_cmd.Data);
v_cmd_data = v_cmd_data(:);

dist_to_wp_time = dist_to_wp.Time(:);
dist_to_wp_data = squeeze(dist_to_wp.Data);
dist_to_wp_data = dist_to_wp_data(:);

capture_event_time = capture_event.Time(:);
capture_event_data = squeeze(capture_event.Data);
capture_event_data = capture_event_data(:);

capture_error_time = capture_error.Time(:);
capture_error_data = squeeze(capture_error.Data);
capture_error_data = capture_error_data(:);

wp_idx_time = wp_idx_out.Time(:);
wp_idx_data = squeeze(wp_idx_out.Data);
wp_idx_data = wp_idx_data(:);


%% Prepare Ground-Truth Pose Data
%
% pose_true is expected to contain:
%
% [x, y, theta]

pose_true_time = pose_true.Time(:);
pose_true_data = squeeze(pose_true.Data);

% Arrange data so rows correspond to time samples.
%
% Supports:
%   N x 3
% or
%   3 x N

if size(pose_true_data,1) ~= length(pose_true_time) && ...
        size(pose_true_data,2) == length(pose_true_time)

    pose_true_data = pose_true_data.';

end

if size(pose_true_data,2) < 3

    error(['pose_true does not contain enough states. ' ...
           'Expected [x, y, theta].']);

end

true_x = pose_true_data(:,1);
true_y = pose_true_data(:,2);
true_theta = pose_true_data(:,3);

true_x = true_x(:);
true_y = true_y(:);
true_theta = true_theta(:);


%% Mission Performance

mission_time = dist_to_wp_time(end);

% Total planned route length calculated directly from wp_plot
route_length = ...
    sum(vecnorm(diff(wp_plot), 2, 2));

if route_length > eps

    % Mission time normalised by planned route length
    mission_time_per_metre = ...
        mission_time / route_length;

    % Effective average progress along planned route
    mission_effective_speed = ...
        route_length / mission_time;

else

    mission_time_per_metre = NaN;
    mission_effective_speed = NaN;

end


fprintf('\n========================================\n');
fprintf('EGH446 Controller Performance\n');
fprintf('========================================\n');

fprintf('\nMission Performance\n');
fprintf('-------------------\n');

fprintf( ...
    'Mission Time                = %.3f s\n', ...
    mission_time);

fprintf( ...
    'Planned Route Length        = %.3f m\n', ...
    route_length);

fprintf( ...
    'Mission Time per Metre      = %.4f s/m\n', ...
    mission_time_per_metre);

fprintf( ...
    'Effective Mission Speed     = %.3f m/s\n', ...
    mission_effective_speed);


%% Desired and Estimated Heading
%
% unwrap() is used ONLY for plotting so artificial +/-180 degree
% discontinuities are removed from the display.
%
% The actual controller continues to use wrapped heading error.

t_heading = theta_hat_time;

% Interpolate wrapped controller heading error onto theta_hat time base
e_interp = interp1( ...
    e_wrap_time, ...
    e_wrap_data, ...
    t_heading, ...
    'linear', ...
    'extrap');

e_interp = e_interp(:);

% Remove artificial +/-pi jumps
theta_hat_unwrapped = ...
    unwrap(theta_hat_data);

% Construct equivalent continuous commanded heading
theta_cmd_continuous = ...
    theta_hat_unwrapped + e_interp;


figure;

plot( ...
    t_heading, ...
    rad2deg(theta_cmd_continuous), ...
    'LineWidth', 1.4);

hold on;

plot( ...
    t_heading, ...
    rad2deg(theta_hat_unwrapped), ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Heading (deg)');
title('Desired and Estimated Vehicle Heading');

legend( ...
    '\theta_{cmd}', ...
    '\theta_{hat}', ...
    'Location', 'best');


%% Wrapped Controller Heading Tracking Error

figure;

plot( ...
    e_wrap_time, ...
    rad2deg(e_wrap_data), ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Heading Error (deg)');
title('Wrapped Controller Heading Tracking Error');


%% Controller Commands

figure;

tiledlayout(2,1);


% Linear velocity command

nexttile;

plot( ...
    v_cmd_time, ...
    v_cmd_data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('v_{cmd} (m/s)');
title('Linear Velocity Command');


% Yaw-rate command

nexttile;

plot( ...
    omega_cmd_time, ...
    omega_cmd_data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('\omega_{cmd} (rad/s)');
title('Yaw Rate Command');


%% Distance to Active Waypoint

figure;

plot( ...
    dist_to_wp_time, ...
    dist_to_wp_data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Distance (m)');
title('Estimated Distance to Active Waypoint');


%% Controller Heading Error Statistics
%
% This is the heading error actually seen by the controller:
%
% theta_cmd - theta_hat

heading_error_deg = ...
    rad2deg(e_wrap_data);

heading_error_rms = ...
    sqrt(mean(heading_error_deg.^2));

heading_error_mean_abs = ...
    mean(abs(heading_error_deg));

heading_error_max_abs = ...
    max(abs(heading_error_deg));


fprintf('\nController Heading Tracking\n');
fprintf('---------------------------\n');

fprintf( ...
    'RMS Heading Error          = %.3f deg\n', ...
    heading_error_rms);

fprintf( ...
    'Mean Absolute Error        = %.3f deg\n', ...
    heading_error_mean_abs);

fprintf( ...
    'Maximum Absolute Error     = %.3f deg\n', ...
    heading_error_max_abs);


%% Ground-Truth Heading Error
%
% Compare commanded heading against the TRUE vehicle heading.

theta_cmd_true_time = interp1( ...
    theta_cmd_time, ...
    theta_cmd_data, ...
    pose_true_time, ...
    'linear', ...
    'extrap');

theta_cmd_true_time = ...
    theta_cmd_true_time(:);


% Wrapped true heading error

true_heading_error = atan2( ...
    sin(theta_cmd_true_time - true_theta), ...
    cos(theta_cmd_true_time - true_theta));

true_heading_error_deg = ...
    rad2deg(true_heading_error);


true_heading_error_rms = ...
    sqrt(mean(true_heading_error_deg.^2));

true_heading_error_mean_abs = ...
    mean(abs(true_heading_error_deg));

true_heading_error_max_abs = ...
    max(abs(true_heading_error_deg));


fprintf('\nGround-Truth Heading Tracking\n');
fprintf('-----------------------------\n');

fprintf( ...
    'TRUE RMS Heading Error     = %.3f deg\n', ...
    true_heading_error_rms);

fprintf( ...
    'TRUE Mean Absolute Error   = %.3f deg\n', ...
    true_heading_error_mean_abs);

fprintf( ...
    'TRUE Maximum Error         = %.3f deg\n', ...
    true_heading_error_max_abs);


%% Kalman State-Estimator Accuracy
%
% Compare Kalman-filtered state estimate x_hat against simulator
% ground truth.
%
% State order is assumed to be:
%
% [x, y, theta]

x_hat_time = ...
    x_hat.Time(:);

x_hat_data = ...
    squeeze(x_hat.Data);


% Arrange estimator data so rows correspond to time samples

if size(x_hat_data,1) ~= length(x_hat_time) && ...
        size(x_hat_data,2) == length(x_hat_time)

    x_hat_data = ...
        x_hat_data.';

end


if size(x_hat_data,2) < 3

    error(['x_hat does not contain enough states. ' ...
           'Expected at least [x, y, theta].']);

end


%% Common Estimator / Ground-Truth Time Range

valid_est_time = ...
    pose_true_time >= x_hat_time(1) & ...
    pose_true_time <= x_hat_time(end);

t_est_compare = ...
    pose_true_time(valid_est_time);

true_x_compare = ...
    true_x(valid_est_time);

true_y_compare = ...
    true_y(valid_est_time);

true_theta_compare = ...
    true_theta(valid_est_time);


%% Interpolate State Estimate Onto Ground-Truth Time Base

x_hat_interp = interp1( ...
    x_hat_time, ...
    x_hat_data(:,1), ...
    t_est_compare, ...
    'linear');

y_hat_interp = interp1( ...
    x_hat_time, ...
    x_hat_data(:,2), ...
    t_est_compare, ...
    'linear');

theta_hat_interp = interp1( ...
    x_hat_time, ...
    x_hat_data(:,3), ...
    t_est_compare, ...
    'linear');


% Force column vectors

x_hat_interp = x_hat_interp(:);
y_hat_interp = y_hat_interp(:);
theta_hat_interp = theta_hat_interp(:);


%% Estimator Errors

x_est_error = ...
    x_hat_interp - true_x_compare;

y_est_error = ...
    y_hat_interp - true_y_compare;


% Wrap heading-estimation error to +/-pi

theta_est_error = atan2( ...
    sin(theta_hat_interp - true_theta_compare), ...
    cos(theta_hat_interp - true_theta_compare));


%% Remove Invalid Estimator Samples

valid_estimator = ...
    isfinite(x_est_error) & ...
    isfinite(y_est_error) & ...
    isfinite(theta_est_error);

x_est_error = ...
    x_est_error(valid_estimator);

y_est_error = ...
    y_est_error(valid_estimator);

theta_est_error = ...
    theta_est_error(valid_estimator);

t_est_valid = ...
    t_est_compare(valid_estimator);


%% Estimator RMSE

if ~isempty(x_est_error)

    estimator_rmse_x = ...
        sqrt(mean(x_est_error.^2));

    estimator_rmse_y = ...
        sqrt(mean(y_est_error.^2));

    % Overall 2-D position RMSE
    estimator_rmse_position = ...
        sqrt(mean( ...
        x_est_error.^2 + ...
        y_est_error.^2));

    % Heading RMSE in degrees
    estimator_rmse_heading = ...
        rad2deg( ...
        sqrt(mean(theta_est_error.^2)));

else

    estimator_rmse_x = NaN;
    estimator_rmse_y = NaN;
    estimator_rmse_position = NaN;
    estimator_rmse_heading = NaN;

end


fprintf('\nState Estimator Accuracy\n');
fprintf('------------------------\n');

fprintf( ...
    'X Position RMSE             = %.3f m\n', ...
    estimator_rmse_x);

fprintf( ...
    'Y Position RMSE             = %.3f m\n', ...
    estimator_rmse_y);

fprintf( ...
    '2-D Position RMSE           = %.3f m\n', ...
    estimator_rmse_position);

fprintf( ...
    'Heading Estimate RMSE       = %.3f deg\n', ...
    estimator_rmse_heading);


%% Estimator Error Plot

if ~isempty(t_est_valid)

    figure;

    tiledlayout(3,1);


    nexttile;

    plot( ...
        t_est_valid, ...
        x_est_error, ...
        'LineWidth', 1.1);

    grid on;

    xlabel('Time (s)');
    ylabel('X Error (m)');
    title('Kalman X-Position Estimation Error');


    nexttile;

    plot( ...
        t_est_valid, ...
        y_est_error, ...
        'LineWidth', 1.1);

    grid on;

    xlabel('Time (s)');
    ylabel('Y Error (m)');
    title('Kalman Y-Position Estimation Error');


    nexttile;

    plot( ...
        t_est_valid, ...
        rad2deg(theta_est_error), ...
        'LineWidth', 1.1);

    grid on;

    xlabel('Time (s)');
    ylabel('Heading Error (deg)');
    title('Kalman Heading Estimation Error');

end


%% Controller Command Statistics

max_v_cmd = ...
    max(v_cmd_data);

mean_v_cmd = ...
    mean(v_cmd_data);

max_omega_cmd = ...
    max(abs(omega_cmd_data));

mean_abs_omega_cmd = ...
    mean(abs(omega_cmd_data));


fprintf('\nController Commands\n');
fprintf('-------------------\n');

fprintf( ...
    'Maximum v_cmd              = %.3f m/s\n', ...
    max_v_cmd);

fprintf( ...
    'Mean v_cmd                 = %.3f m/s\n', ...
    mean_v_cmd);

fprintf( ...
    'Maximum |omega_cmd|        = %.3f rad/s\n', ...
    max_omega_cmd);

fprintf( ...
    'Mean |omega_cmd|           = %.3f rad/s\n', ...
    mean_abs_omega_cmd);


%% Yaw-Rate Saturation Analysis
%
% Controller is considered saturated when |omega_cmd| reaches
% at least 99% of omega_max.

yaw_sat_threshold = ...
    0.99 * omega_max;

yaw_is_saturated = ...
    abs(omega_cmd_data) >= yaw_sat_threshold;


% Time-weighted saturation calculation

if length(omega_cmd_time) > 1

    yaw_saturated_time = ...
        trapz( ...
        omega_cmd_time, ...
        double(yaw_is_saturated));

    yaw_total_time = ...
        omega_cmd_time(end) - ...
        omega_cmd_time(1);


    if yaw_total_time > 0

        yaw_saturation_percent = ...
            100 * ...
            yaw_saturated_time / ...
            yaw_total_time;

    else

        yaw_saturation_percent = NaN;

    end

else

    yaw_saturated_time = NaN;
    yaw_saturation_percent = NaN;

end


fprintf('\nYaw-Rate Saturation\n');
fprintf('-------------------\n');

fprintf( ...
    'Yaw Saturation Limit        = %.3f rad/s\n', ...
    omega_max);

fprintf( ...
    'Time at Yaw Saturation      = %.3f s\n', ...
    yaw_saturated_time);

fprintf( ...
    'Yaw Saturation Percentage   = %.2f %%\n', ...
    yaw_saturation_percent);


%% Estimated Waypoint Capture Error
%
% Guidance uses x_hat to determine waypoint capture.

captured = ...
    capture_event_data > 0.5;


% Align capture error with capture-event time base if required

if length(capture_error_data) ~= length(capture_event_data) || ...
        any(capture_error_time ~= capture_event_time)

    capture_error_aligned = interp1( ...
        capture_error_time, ...
        capture_error_data, ...
        capture_event_time, ...
        'previous', ...
        'extrap');

else

    capture_error_aligned = ...
        capture_error_data;

end


capture_error_aligned = ...
    capture_error_aligned(:);

waypoint_capture_errors = ...
    capture_error_aligned(captured);


% Remove invalid entries

waypoint_capture_errors = ...
    waypoint_capture_errors( ...
    isfinite(waypoint_capture_errors) & ...
    waypoint_capture_errors >= 0);


num_waypoints_captured = ...
    length(waypoint_capture_errors);


if num_waypoints_captured > 0

    mean_capture_error = ...
        mean(waypoint_capture_errors);

    max_capture_error = ...
        max(waypoint_capture_errors);

    min_capture_error = ...
        min(waypoint_capture_errors);

else

    mean_capture_error = NaN;
    max_capture_error = NaN;
    min_capture_error = NaN;

end


fprintf('\nEstimated Waypoint Capture Performance\n');
fprintf('--------------------------------------\n');

fprintf( ...
    'Waypoints Captured          = %d\n', ...
    num_waypoints_captured);

fprintf( ...
    'Average Capture Error       = %.3f m\n', ...
    mean_capture_error);

fprintf( ...
    'Maximum Capture Error       = %.3f m\n', ...
    max_capture_error);

fprintf( ...
    'Minimum Capture Error       = %.3f m\n', ...
    min_capture_error);


%% Display Individual Estimated Capture Errors

if num_waypoints_captured > 0

    fprintf('\nIndividual Estimated Capture Errors\n');
    fprintf('-----------------------------------\n');

    for i = 1:num_waypoints_captured

        fprintf( ...
            'Waypoint %2d = %.3f m\n', ...
            i, ...
            waypoint_capture_errors(i));

    end

end


%% Ground-Truth Waypoint Capture Error
%
% Guidance decides waypoint capture using x_hat.
%
% Calculate TRUE physical distance between vehicle and waypoint at
% the instant Guidance declares capture.

capture_samples = ...
    find(captured);

true_capture_errors = [];
true_capture_wp_indices = [];


for k = 1:length(capture_samples)

    capture_sample = ...
        capture_samples(k);

    % Time at which Guidance declared capture
    t_cap = ...
        capture_event_time(capture_sample);


    % True vehicle position at capture time
    x_true_cap = interp1( ...
        pose_true_time, ...
        true_x, ...
        t_cap, ...
        'linear', ...
        'extrap');

    y_true_cap = interp1( ...
        pose_true_time, ...
        true_y, ...
        t_cap, ...
        'linear', ...
        'extrap');


    % Active waypoint at capture time
    wp_cap = interp1( ...
        wp_idx_time, ...
        double(wp_idx_data), ...
        t_cap, ...
        'previous', ...
        'extrap');

    wp_cap = ...
        round(wp_cap);


    % Ensure waypoint index is valid
    if wp_cap >= 1 && ...
            wp_cap <= size(wp_ordered,1)

        wp_x = ...
            wp_ordered(wp_cap,1);

        wp_y = ...
            wp_ordered(wp_cap,2);


        true_error = sqrt( ...
            (x_true_cap - wp_x)^2 + ...
            (y_true_cap - wp_y)^2);


        true_capture_errors(end+1,1) = ...
            true_error;

        true_capture_wp_indices(end+1,1) = ...
            wp_cap;

    end

end


%% Remove Invalid Ground-Truth Capture Errors

valid_true_capture = ...
    isfinite(true_capture_errors) & ...
    true_capture_errors >= 0;

true_capture_errors = ...
    true_capture_errors(valid_true_capture);

true_capture_wp_indices = ...
    true_capture_wp_indices(valid_true_capture);


num_true_waypoints_captured = ...
    length(true_capture_errors);


if num_true_waypoints_captured > 0

    mean_true_capture_error = ...
        mean(true_capture_errors);

    max_true_capture_error = ...
        max(true_capture_errors);

    min_true_capture_error = ...
        min(true_capture_errors);

else

    mean_true_capture_error = NaN;
    max_true_capture_error = NaN;
    min_true_capture_error = NaN;

end


fprintf('\nGround-Truth Waypoint Capture Performance\n');
fprintf('-----------------------------------------\n');

fprintf( ...
    'Waypoints Captured          = %d\n', ...
    num_true_waypoints_captured);

fprintf( ...
    'Average TRUE Capture Error  = %.3f m\n', ...
    mean_true_capture_error);

fprintf( ...
    'Maximum TRUE Capture Error  = %.3f m\n', ...
    max_true_capture_error);

fprintf( ...
    'Minimum TRUE Capture Error  = %.3f m\n', ...
    min_true_capture_error);


%% Display Individual Ground-Truth Capture Errors

if num_true_waypoints_captured > 0

    fprintf('\nIndividual TRUE Capture Errors\n');
    fprintf('------------------------------\n');

    for i = 1:num_true_waypoints_captured

        fprintf( ...
            'Waypoint %2d = %.3f m\n', ...
            true_capture_wp_indices(i), ...
            true_capture_errors(i));

    end

end


%% Ground-Truth Finite-Segment Cross-Track Error
%
% wp_plot contains:
%
% Row 1  = vehicle starting position
% Row 2  = waypoint 1
% Row 3  = waypoint 2
% ...
%
% While targeting waypoint i:
%
% wp_plot(i,:) -> wp_plot(i+1,:)
%
% Cross-track error is calculated using TRUE simulator position.


%% Match Active Waypoint Index to Ground-Truth Time

wp_index = interp1( ...
    wp_idx_time, ...
    double(wp_idx_data), ...
    pose_true_time, ...
    'previous', ...
    'extrap');

wp_index = ...
    round(wp_index(:));


%% Desired Route

route = ...
    wp_plot;

cross_track_error = ...
    NaN(length(pose_true_time),1);


%% Calculate Finite-Segment Cross-Track Error
%
% Find closest point on ACTIVE finite route segment.

for k = 1:length(pose_true_time)

    i = ...
        wp_index(k);


    if i >= 1 && ...
            i < size(route,1)

        % Start and end of active path segment
        A = ...
            route(i,:);

        B = ...
            route(i+1,:);


        % TRUE vehicle position
        P = ...
            [true_x(k), true_y(k)];


        % Path-segment vector
        segment = ...
            B - A;

        segment_length_sq = ...
            dot(segment, segment);


        if segment_length_sq > eps

            % Projection onto path segment
            t_projection = ...
                dot(P - A, segment) / ...
                segment_length_sq;


            % Clamp projection to finite segment
            t_clamped = ...
                max(0, min(1, t_projection));


            % Closest point on finite segment
            closest_point = ...
                A + ...
                t_clamped * segment;


            % Euclidean ground-truth distance
            cross_track_error(k) = ...
                norm(P - closest_point);

        end

    end

end


%% Remove Invalid Cross-Track Samples

valid_ct = ...
    isfinite(cross_track_error);

cross_track_valid = ...
    cross_track_error(valid_ct);


%% Ground-Truth Finite-Segment Cross-Track Statistics

if ~isempty(cross_track_valid)

    rms_cross_track_error = ...
        sqrt(mean(cross_track_valid.^2));

    mean_cross_track_error = ...
        mean(cross_track_valid);

    max_cross_track_error = ...
        max(cross_track_valid);

else

    rms_cross_track_error = NaN;
    mean_cross_track_error = NaN;
    max_cross_track_error = NaN;

end


fprintf('\nGround-Truth Finite-Segment Cross-Track Performance\n');
fprintf('---------------------------------------------------\n');

fprintf( ...
    'TRUE RMS Cross-Track Error  = %.3f m\n', ...
    rms_cross_track_error);

fprintf( ...
    'TRUE Mean Cross-Track Error = %.3f m\n', ...
    mean_cross_track_error);

fprintf( ...
    'TRUE Maximum CT Error       = %.3f m\n', ...
    max_cross_track_error);


%% Ground-Truth Finite-Segment Cross-Track Error Plot

figure;

plot( ...
    pose_true_time(valid_ct), ...
    cross_track_valid, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Distance to Active Path Segment (m)');
title('Ground-Truth Finite-Segment Cross-Track Error');


%% Final Performance Summary

fprintf('\n========================================\n');
fprintf('Final Performance Summary\n');
fprintf('========================================\n');


fprintf( ...
    'Mission Time                = %.3f s\n', ...
    mission_time);

fprintf( ...
    'Route Length                = %.3f m\n', ...
    route_length);

fprintf( ...
    'Mission Time per Metre      = %.4f s/m\n', ...
    mission_time_per_metre);

fprintf( ...
    'Effective Mission Speed     = %.3f m/s\n', ...
    mission_effective_speed);


fprintf( ...
    'Waypoints Captured          = %d/%d\n', ...
    num_true_waypoints_captured, ...
    size(wp_ordered,1));


fprintf( ...
    'Controller RMS Heading Err  = %.3f deg\n', ...
    heading_error_rms);

fprintf( ...
    'TRUE RMS Heading Error      = %.3f deg\n', ...
    true_heading_error_rms);


fprintf( ...
    'Estimated Capture Error     = %.3f m\n', ...
    mean_capture_error);

fprintf( ...
    'TRUE Capture Error          = %.3f m\n', ...
    mean_true_capture_error);


fprintf( ...
    'TRUE RMS Finite-Segment CT  = %.3f m\n', ...
    rms_cross_track_error);

fprintf( ...
    'TRUE Mean Finite-Segment CT = %.3f m\n', ...
    mean_cross_track_error);

fprintf( ...
    'TRUE Max Finite-Segment CT  = %.3f m\n', ...
    max_cross_track_error);


fprintf( ...
    'Estimator X RMSE            = %.3f m\n', ...
    estimator_rmse_x);

fprintf( ...
    'Estimator Y RMSE            = %.3f m\n', ...
    estimator_rmse_y);

fprintf( ...
    'Estimator Position RMSE     = %.3f m\n', ...
    estimator_rmse_position);

fprintf( ...
    'Estimator Heading RMSE      = %.3f deg\n', ...
    estimator_rmse_heading);


fprintf( ...
    'Mean |omega_cmd|            = %.3f rad/s\n', ...
    mean_abs_omega_cmd);

fprintf( ...
    'Yaw Saturation Time         = %.3f s\n', ...
    yaw_saturated_time);

fprintf( ...
    'Yaw Saturation Percentage   = %.2f %%\n', ...
    yaw_saturation_percent);


fprintf('========================================\n');