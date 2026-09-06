%% close previously open model
close_system('sl_groundvehicleDynamics',0);


%% add toolboxes to path
homedir = pwd;
addpath( genpath(strcat(homedir,[filesep,'toolboxes'])));

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);


%% open current model
open_system('sl_groundvehicleDynamics'); % ground robot model

% StopFcn fires AFTER the simulation (and animation) finishes, so all
% analysis figures appear once the run is complete. One call, one file.
set_param('sl_groundvehicleDynamics', 'StopFcn', 'mission_analysis();')

cd(homedir);
