function controller_params()
% CONTROLLER_PARAMS Initialise controller tuning parameters
%
% Places controller parameters into the MATLAB base workspace so they
% can be accessed by the Simulink controller blocks.

%% Controller Parameters

% Heading controller proportional gain
K_theta = 1;

% Distance-based speed gain
K_dist = 0.5;

% Maximum forward velocity [m/s]
v_max = 2;

% Maximum yaw rate [rad/s]
omega_max = pi/2;

%% Export parameters to base workspace
assignin('base', 'K_theta', K_theta);
assignin('base', 'K_dist', K_dist);
assignin('base', 'v_max', v_max);
assignin('base', 'omega_max', omega_max);

end