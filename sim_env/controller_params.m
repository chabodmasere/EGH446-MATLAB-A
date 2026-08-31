function controller_params()
% CONTROLLER_PARAMS Initialise controller tuning parameters
%
% Places controller parameters into the MATLAB base workspace so they
% can be accessed by the Simulink controller blocks.

%% Controller Parameters

% Heading controller proportional gain
K_theta = 1;

% Distance-based speed gain
K_dist = 1;

% Maximum forward velocity [m/s]
v_max = 1;

% Maximum yaw rate [rad/s]
omega_max = pi/2;

% Min heading speed for above 90 degree turns
heading_speed_min = 0.15;

%% Export parameters to base workspace
assignin('base', 'K_theta', K_theta);
assignin('base', 'K_dist', K_dist);
assignin('base', 'v_max', v_max);
assignin('base', 'omega_max', omega_max);
assignin('base','heading_speed_min',heading_speed_min);

end