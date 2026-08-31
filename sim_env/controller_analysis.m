%% EGH446 Controller Analysis
% Analyse logged controller and navigation signals after Simulink simulation
%
% This version forces scalar logged signals into column vectors to avoid
% accidental NxN implicit expansion after Kalman-filter integration.

%% Clear large analysis variables from previous failed runs

clear theta_cmd_continuous theta_hat_unwrapped e_interp heading_error_deg
clear cross_track_error cross_track_valid vehicle_state


%% Extract logged signals

theta_cmd  = logsout.getElement('theta_cmd').Values;
theta_hat  = logsout.getElement('theta_hat').Values;
e_wrap     = logsout.getElement('e_wrap').Values;
omega_cmd  = logsout.getElement('omega_cmd').Values;
v_cmd      = logsout.getElement('v_cmd').Values;
dist_to_wp = logsout.getElement('dist_to_wp').Values;

% Additional navigation-analysis signals
x_hat         = logsout.getElement('x_hat').Values;
wp_idx_out    = logsout.getElement('wp_idx_out').Values;
capture_event = logsout.getElement('capture_event').Values;
capture_error = logsout.getElement('capture_error').Values;


%% Convert scalar logged signals to column vectors
% This prevents row-vector/column-vector implicit expansion.

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


%% Mission Time

mission_time = dist_to_wp_time(end);

fprintf('\n========================================\n');
fprintf('EGH446 Controller Performance\n');
fprintf('========================================\n');

fprintf('\nMission Performance\n');
fprintf('-------------------\n');
fprintf('Mission Time = %.3f s\n', mission_time);


%% Desired and Estimated Heading
% The Kalman-filter heading may wrap at +/-pi.
%
% unwrap() is used ONLY for plotting so artificial +/-180 degree jumps
% do not appear in the desired/estimated heading figure.
%
% The actual controller continues to use the wrapped heading error.

t_heading = theta_hat_time;

% Interpolate wrapped heading error onto theta_hat time base
e_interp = interp1( ...
    e_wrap_time, ...
    e_wrap_data, ...
    t_heading, ...
    'linear', ...
    'extrap');

% Force interpolation result to column vector
e_interp = e_interp(:);

% Remove artificial +/-pi jumps from estimated heading for plotting
theta_hat_unwrapped = unwrap(theta_hat_data);

% Construct equivalent commanded heading in the same continuous
% angular representation as the unwrapped estimated heading
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


%% Wrapped Heading Tracking Error

figure;

plot( ...
    e_wrap_time, ...
    rad2deg(e_wrap_data), ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Heading Error (deg)');
title('Wrapped Heading Tracking Error');


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
title('Distance to Active Waypoint');


%% Heading Error Statistics

heading_error_deg = rad2deg(e_wrap_data);

heading_error_rms = ...
    sqrt(mean(heading_error_deg.^2));

heading_error_mean_abs = ...
    mean(abs(heading_error_deg));

heading_error_max_abs = ...
    max(abs(heading_error_deg));

fprintf('\nHeading Tracking\n');
fprintf('----------------\n');

fprintf( ...
    'RMS Heading Error          = %.3f deg\n', ...
    heading_error_rms);

fprintf( ...
    'Mean Absolute Error        = %.3f deg\n', ...
    heading_error_mean_abs);

fprintf( ...
    'Maximum Absolute Error     = %.3f deg\n', ...
    heading_error_max_abs);


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


%% Waypoint Capture Error

% Find samples where a waypoint capture occurred
captured = capture_event_data > 0.5;

% If capture_event and capture_error have different logged time bases,
% interpolate capture error onto the capture-event time base.
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

% Remove invalid entries if present
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


fprintf('\nWaypoint Capture Performance\n');
fprintf('----------------------------\n');

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


%% Display Individual Waypoint Capture Errors

if num_waypoints_captured > 0

    fprintf('\nIndividual Capture Errors\n');
    fprintf('-------------------------\n');

    for i = 1:num_waypoints_captured

        fprintf( ...
            'Waypoint %2d = %.3f m\n', ...
            i, ...
            waypoint_capture_errors(i));

    end

end


%% RMS Cross-Track Error
%
% wp_plot is assumed to contain:
%
% Row 1  = vehicle starting position
% Row 2  = waypoint 1
% Row 3  = waypoint 2
% ...
% Row 11 = waypoint 10
%
% Therefore, while targeting waypoint i, the desired path segment is:
%
% wp_plot(i,:) -> wp_plot(i+1,:)


%% Prepare Vehicle-State Data

vehicle_state = squeeze(x_hat.Data);
t_vehicle = x_hat.Time(:);

% Arrange state matrix so rows correspond to time samples.
%
% This supports both:
%
% N x states
%
% and
%
% states x N

if size(vehicle_state,1) ~= length(t_vehicle) && ...
        size(vehicle_state,2) == length(t_vehicle)

    vehicle_state = vehicle_state.';

end

% Check that the state estimate contains at least x and y
if size(vehicle_state,2) < 2

    error(['x_hat does not contain enough states for cross-track ' ...
           'analysis. Expected at least x and y.']);

end

vehicle_x = vehicle_state(:,1);
vehicle_y = vehicle_state(:,2);

vehicle_x = vehicle_x(:);
vehicle_y = vehicle_y(:);


%% Match Active Waypoint Index to Vehicle Time

wp_index = interp1( ...
    wp_idx_time, ...
    double(wp_idx_data), ...
    t_vehicle, ...
    'previous', ...
    'extrap');

wp_index = round(wp_index(:));


%% Desired Route

route = wp_plot;

cross_track_error = ...
    NaN(length(t_vehicle),1);


%% Calculate Cross-Track Error

for k = 1:length(t_vehicle)

    i = wp_index(k);

    % Check that the corresponding path segment exists
    if i >= 1 && i < size(route,1)

        % Beginning and end of desired path segment
        A = route(i,:);
        B = route(i+1,:);

        % Current estimated vehicle position
        P = [vehicle_x(k), vehicle_y(k)];

        % Desired path segment vector
        segment = B - A;

        segment_length = norm(segment);

        if segment_length > eps

            % Signed perpendicular distance between the vehicle
            % and the desired path line
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


%% Cross-Track Statistics

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


fprintf('\nCross-Track Performance\n');
fprintf('-----------------------\n');

fprintf( ...
    'RMS Cross-Track Error       = %.3f m\n', ...
    rms_cross_track_error);

fprintf( ...
    'Mean Absolute Cross-Track   = %.3f m\n', ...
    mean_abs_cross_track_error);

fprintf( ...
    'Maximum Cross-Track Error   = %.3f m\n', ...
    max_abs_cross_track_error);


%% Cross-Track Error Plot

figure;

plot( ...
    t_vehicle(valid_ct), ...
    cross_track_valid, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Cross-Track Error (m)');
title('Vehicle Cross-Track Error');


%% Final Performance Summary

fprintf('\n========================================\n');
fprintf('Final Performance Summary\n');
fprintf('========================================\n');

fprintf( ...
    'Mission Time                = %.3f s\n', ...
    mission_time);

fprintf( ...
    'RMS Heading Error           = %.3f deg\n', ...
    heading_error_rms);

fprintf( ...
    'Average Capture Error       = %.3f m\n', ...
    mean_capture_error);

fprintf( ...
    'RMS Cross-Track Error       = %.3f m\n', ...
    rms_cross_track_error);

fprintf('========================================\n');