%% EGH446 Controller Analysis
% Analyse logged controller and navigation signals after Simulink simulation

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


%% Mission Time

mission_time = dist_to_wp.Time(end);

fprintf('\n========================================\n');
fprintf('EGH446 Controller Performance\n');
fprintf('========================================\n');

fprintf('\nMission Performance\n');
fprintf('-------------------\n');
fprintf('Mission Time = %.3f s\n', mission_time);


%% Desired and Estimated Heading
% theta_hat may accumulate outside +/-pi.
% Construct an equivalent commanded heading near theta_hat using the
% wrapped heading error. This avoids misleading +/-180 degree jumps.

t_heading = theta_hat.Time;

% Interpolate wrapped error onto theta_hat time base if required
e_interp = interp1( ...
    e_wrap.Time, ...
    e_wrap.Data, ...
    t_heading, ...
    'linear', ...
    'extrap');

% Equivalent commanded heading using same continuous angle representation
theta_cmd_continuous = theta_hat.Data + e_interp;

figure;

plot( ...
    t_heading, ...
    rad2deg(theta_cmd_continuous), ...
    'LineWidth', 1.4);

hold on;

plot( ...
    t_heading, ...
    rad2deg(theta_hat.Data), ...
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
    e_wrap.Time, ...
    rad2deg(e_wrap.Data), ...
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
    v_cmd.Time, ...
    v_cmd.Data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('v_{cmd} (m/s)');
title('Linear Velocity Command');


% Yaw-rate command
nexttile;

plot( ...
    omega_cmd.Time, ...
    omega_cmd.Data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('\omega_{cmd} (rad/s)');
title('Yaw Rate Command');


%% Distance to Active Waypoint

figure;

plot( ...
    dist_to_wp.Time, ...
    dist_to_wp.Data, ...
    'LineWidth', 1.2);

grid on;

xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance to Active Waypoint');


%% Heading Error Statistics

heading_error_deg = rad2deg(e_wrap.Data);

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

max_v_cmd = max(v_cmd.Data);

mean_v_cmd = mean(v_cmd.Data);

max_omega_cmd = max(abs(omega_cmd.Data));

mean_abs_omega_cmd = mean(abs(omega_cmd.Data));

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

capture_events = squeeze(capture_event.Data);
capture_errors = squeeze(capture_error.Data);

% Find samples where a waypoint capture occurred
captured = capture_events > 0.5;

waypoint_capture_errors = capture_errors(captured);

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


%% Prepare vehicle-state data

vehicle_state = squeeze(x_hat.Data);

% Ensure state matrix is arranged as:
% [number of samples x number of states]

if size(vehicle_state,2) ~= 3 && ...
        size(vehicle_state,1) == 3

    vehicle_state = vehicle_state.';

end

vehicle_x = vehicle_state(:,1);
vehicle_y = vehicle_state(:,2);

t_vehicle = x_hat.Time;


%% Match active waypoint index to vehicle time

wp_index_data = double(squeeze(wp_idx_out.Data));

wp_index = interp1( ...
    wp_idx_out.Time, ...
    wp_index_data, ...
    t_vehicle, ...
    'previous', ...
    'extrap');

wp_index = round(wp_index);


%% Desired route

route = wp_plot;

cross_track_error = ...
    NaN(length(t_vehicle),1);


%% Calculate cross-track error

for k = 1:length(t_vehicle)

    i = wp_index(k);

    % Check that the corresponding path segment exists
    if i >= 1 && i < size(route,1)

        % Beginning and end of desired path segment
        A = route(i,:);
        B = route(i+1,:);

        % Current vehicle position
        P = [vehicle_x(k), vehicle_y(k)];

        % Path segment vector
        segment = B - A;

        segment_length = norm(segment);

        if segment_length > eps

            % Signed perpendicular distance between the vehicle
            % and the desired path segment line
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


%% Remove invalid cross-track samples

valid_ct = ...
    isfinite(cross_track_error);

cross_track_valid = ...
    cross_track_error(valid_ct);


%% Cross-track statistics

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