%% EGH446 Controller Analysis
% Analyse logged controller, guidance and vehicle signals after simulation.
%
% The controller continues to operate using the Kalman-filtered estimate
% x_hat. Ground-truth vehicle pose is used ONLY for offline performance
% evaluation.

close all;

%% Clear analysis variables from previous runs

clear theta_cmd_continuous theta_hat_unwrapped e_interp
clear heading_error_deg true_heading_error_deg
clear cross_track_error cross_track_valid
clear true_capture_errors


%% Extract logged signals

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
% Supports both:
%
% N x 3
%
% and
%
% 3 x N

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


%% Mission Time

mission_time = dist_to_wp_time(end);

fprintf('\n========================================\n');
fprintf('EGH446 Controller Performance\n');
fprintf('========================================\n');

fprintf('\nMission Performance\n');
fprintf('-------------------\n');
fprintf('Mission Time = %.3f s\n', mission_time);


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
theta_hat_unwrapped = unwrap(theta_hat_data);

% Construct equivalent continuous commanded heading
theta_cmd_continuous = theta_hat_unwrapped + e_interp;

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

heading_error_deg = rad2deg(e_wrap_data);

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
%
% theta_cmd is interpolated onto the ground-truth pose time base.

theta_cmd_true_time = interp1( ...
    theta_cmd_time, ...
    theta_cmd_data, ...
    pose_true_time, ...
    'linear', ...
    'extrap');

theta_cmd_true_time = theta_cmd_true_time(:);

% Wrapped true heading error
true_heading_error = atan2( ...
    sin(theta_cmd_true_time - true_theta), ...
    cos(theta_cmd_true_time - true_theta));

true_heading_error_deg = rad2deg(true_heading_error);

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


%% Controller Command Statistics

max_v_cmd = max(v_cmd_data);

mean_v_cmd = mean(v_cmd_data);

max_omega_cmd = max(abs(omega_cmd_data));

mean_abs_omega_cmd = mean(abs(omega_cmd_data));

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


%% Estimated Waypoint Capture Error
%
% These values are produced by Guidance using x_hat.
%
% They represent the waypoint distance estimated by the navigation system
% when the waypoint capture condition was triggered.

captured = capture_event_data > 0.5;

% Align capture error with capture-event time base if necessary
if length(capture_error_data) ~= length(capture_event_data) || ...
        any(capture_error_time ~= capture_event_time)

    capture_error_aligned = interp1( ...
        capture_error_time, ...
        capture_error_data, ...
        capture_event_time, ...
        'previous', ...
        'extrap');

else

    capture_error_aligned = capture_error_data;

end

capture_error_aligned = capture_error_aligned(:);

waypoint_capture_errors = ...
    capture_error_aligned(captured);

% Remove invalid entries
waypoint_capture_errors = waypoint_capture_errors( ...
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
% Here we calculate the TRUE physical distance between the vehicle
% and the active waypoint at the instant that Guidance triggered
% capture_event.

capture_samples = find(captured);

true_capture_errors = [];
true_capture_wp_indices = [];


for k = 1:length(capture_samples)

    capture_sample = capture_samples(k);

    % Time at which Guidance declared waypoint capture
    t_cap = capture_event_time(capture_sample);

    % True vehicle position at that instant
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

    % Determine which waypoint was active at capture
    wp_cap = interp1( ...
        wp_idx_time, ...
        double(wp_idx_data), ...
        t_cap, ...
        'previous', ...
        'extrap');

    wp_cap = round(wp_cap);

    % Ensure waypoint index is valid
    if wp_cap >= 1 && wp_cap <= size(wp_ordered,1)

        wp_x = wp_ordered(wp_cap,1);
        wp_y = wp_ordered(wp_cap,2);

        true_error = sqrt( ...
            (x_true_cap - wp_x)^2 + ...
            (y_true_cap - wp_y)^2 );

        true_capture_errors(end+1,1) = true_error;
        true_capture_wp_indices(end+1,1) = wp_cap;

    end

end


% Remove invalid values
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


%% Ground-Truth RMS Cross-Track Error
%
% wp_plot contains:
%
% Row 1  = vehicle starting position
% Row 2  = waypoint 1
% Row 3  = waypoint 2
% ...
% Row 11 = waypoint 10
%
% While targeting waypoint i:
%
% wp_plot(i,:) -> wp_plot(i+1,:)
%
% IMPORTANT:
% Cross-track error is now calculated using the TRUE simulator position,
% not the Kalman-filtered position estimate.


%% Match Active Waypoint Index to Ground-Truth Time

wp_index = interp1( ...
    wp_idx_time, ...
    double(wp_idx_data), ...
    pose_true_time, ...
    'previous', ...
    'extrap');

wp_index = round(wp_index(:));


%% Desired Route

route = wp_plot;

cross_track_error = ...
    NaN(length(pose_true_time),1);


%% Calculate Ground-Truth Cross-Track Error

for k = 1:length(pose_true_time)

    i = wp_index(k);

    % Check that corresponding desired path segment exists
    if i >= 1 && i < size(route,1)

        % Beginning and end of desired path segment
        A = route(i,:);
        B = route(i+1,:);

        % TRUE vehicle position
        P = [true_x(k), true_y(k)];

        % Desired path vector
        segment = B - A;

        segment_length = norm(segment);

        if segment_length > eps

            % Signed perpendicular distance between true vehicle
            % position and desired path line
            cross_track_error(k) = ...
                ( ...
                segment(1)*(P(2)-A(2)) ...
                - ...
                segment(2)*(P(1)-A(1)) ...
                ) ...
                / segment_length;

        end

    end

end


%% Remove Invalid Cross-Track Samples

valid_ct = ...
    isfinite(cross_track_error);

cross_track_valid = ...
    cross_track_error(valid_ct);


%% Ground-Truth Cross-Track Statistics

if ~isempty(cross_track_valid)

    rms_cross_track_error = ...
        sqrt(mean(cross_track_valid.^2));

    mean_abs_cross_track_error = ...
        mean(abs(cross_track_valid));

    max_abs_cross_track_error = ...
        max(abs(cross_track_valid));

else

    rms_cross_track_error = NaN;
    mean_abs_cross_track_error = NaN;
    max_abs_cross_track_error = NaN;

end


fprintf('\nGround-Truth Cross-Track Performance\n');
fprintf('------------------------------------\n');

fprintf( ...
    'TRUE RMS Cross-Track Error  = %.3f m\n', ...
    rms_cross_track_error);

fprintf( ...
    'TRUE Mean Absolute CT Error = %.3f m\n', ...
    mean_abs_cross_track_error);

fprintf( ...
    'TRUE Maximum CT Error       = %.3f m\n', ...
    max_abs_cross_track_error);


%% Ground-Truth Cross-Track Error Plot

figure;

plot( ...
    pose_true_time(valid_ct), ...
    cross_track_valid, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Cross-Track Error (m)');
title('Ground-Truth Vehicle Cross-Track Error');


%% Final Performance Summary

fprintf('\n========================================\n');
fprintf('Final Performance Summary\n');
fprintf('========================================\n');

fprintf( ...
    'Mission Time                = %.3f s\n', ...
    mission_time);

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
    'TRUE RMS Cross-Track Error  = %.3f m\n', ...
    rms_cross_track_error);

fprintf('========================================\n');