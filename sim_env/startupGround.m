%% close previously open model
close_system('sl_groundvehicleDynamics', 0);

%% add toolboxes to path
homedir = pwd;
addpath(genpath(fullfile(homedir, 'toolboxes')));
cd(fullfile('toolboxes', 'MRTB'));
startMobileRoboticsSimulationToolbox;
cd(homedir);

%% load tuning parameters into the base workspace
controller_params();      % K_theta, K_dist, v_max, omega_max, heading_speed_min

%% open model, clear any saved post-run callback (analysis is run manually)
open_system('sl_groundvehicleDynamics');
set_param('sl_groundvehicleDynamics', 'StopFcn', '');