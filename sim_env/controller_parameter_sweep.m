%% EGH446 Final Integrated Controller Parameter Sweep
%
% Controlled one-variable-at-a-time parameter sweep for the FINAL
% integrated EGH446 ground-vehicle system.
%
% PURPOSE
% -------
% This script is intended for FINAL report tuning tables.
%
% Unlike robustness testing, every parameter value is tested using:
%
%   - the SAME initial vehicle pose;
%   - the SAME waypoint set;
%   - the SAME optimised waypoint order;
%   - the SAME final guidance implementation;
%   - the SAME Kalman/state-estimation implementation;
%   - the SAME vehicle model;
%   - the SAME random-number seed where applicable.
%
% Only ONE selected parameter is changed between simulations.
%
%
% IMPORTANT
% ---------
% The model's normal InitFcn may contain:
%
%   mdl_robot;
%   controller_params;
%
% This script temporarily replaces the InitFcn while the sweep is running.
% The original InitFcn is automatically restored when the script finishes
% or if an error occurs.
%
%
% FIRST RUN
% ---------
% On the first execution, a fixed mission is generated and saved as:
%
%   controller_sweep_fixed_mission.mat
%
% ALL later parameter sweeps reuse this same mission.
%
% To deliberately create a new tuning mission, delete:
%
%   controller_sweep_fixed_mission.mat
%
% and run this script again.
%
%
% REQUIRED
% --------
% controller_analysis.m must be the FINAL version containing the
% ground-truth performance calculations used by final_robustness_testing.


clear;
clc;
close all;


%% ========================================================================
%  SELECT PARAMETER TO SWEEP
%  ========================================================================
%
% COMMENT OUT every sweep except ONE.
%
% The values shown below are suggested test sets based on the controller
% development completed so far. They can be edited before running.


% -------------------------------------------------------------------------
% 1. HEADING PROPORTIONAL GAIN
% -------------------------------------------------------------------------

%sweep_parameter = 'K_theta';
%sweep_values = [0.5 1.0 2.0 4.0];


% -------------------------------------------------------------------------
% 2. DISTANCE / LINEAR VELOCITY GAIN
% -------------------------------------------------------------------------

%sweep_parameter = 'K_dist';
%sweep_values = [0.25 0.50 0.75 1.00];


% -------------------------------------------------------------------------
% 3. MAXIMUM LINEAR VELOCITY
% -------------------------------------------------------------------------

%sweep_parameter = 'v_max';
%sweep_values = [1.5 2.0 2.5 3.0 3.5];


% -------------------------------------------------------------------------
% 4. MAXIMUM YAW RATE
% -------------------------------------------------------------------------

sweep_parameter = 'omega_max';
sweep_values = [pi/4 pi/3 2*pi/5 pi/2 2*pi/3];


% -------------------------------------------------------------------------
% 5. MINIMUM HEADING-DEPENDENT SPEED FACTOR
% -------------------------------------------------------------------------

%sweep_parameter = 'heading_speed_min';
%sweep_values = [0.10 0.15 0.25 0.40];


% -------------------------------------------------------------------------
% 6. RVWP LOOKAHEAD DISTANCE
%
% EDIT these values if your final RVWP operating range differs.
% -------------------------------------------------------------------------

%sweep_parameter = 'rvwp_lookahead';
%sweep_values = [2 5 10 15];


% -------------------------------------------------------------------------
% 7. WAYPOINT CAPTURE RADIUS
%
% Useful for the navigation-accuracy vs mission-time trade-off.
% -------------------------------------------------------------------------

%sweep_parameter = 'capture_radius';
%sweep_values = [0.5 1.0 1.5 2.0 3.0];


%% ========================================================================
%  GENERAL SETTINGS
%  ========================================================================

% Simulink model name WITHOUT .slx
modelName = 'sl_groundvehicleDynamics';

% File used to preserve exactly the same mission between every sweep
fixedMissionFile = 'controller_sweep_fixed_mission.mat';

% Reuse the same MATLAB random-number sequence for every simulation.
% This helps prevent stochastic differences from contaminating the sweep.
fixed_rng_seed = 446;

% false = suppress controller_analysis figures/output during sweep
% true  = display controller_analysis output for every simulation
show_detailed_run_output = false;


%% ========================================================================
%  VALIDATE SWEEP SELECTION
%  ========================================================================

valid_parameters = { ...
    'K_theta', ...
    'K_dist', ...
    'v_max', ...
    'omega_max', ...
    'heading_speed_min', ...
    'rvwp_lookahead', ...
    'capture_radius'};


if ~ismember(sweep_parameter, valid_parameters)

    error( ...
        'Unknown sweep parameter: %s', ...
        sweep_parameter);

end


if isempty(sweep_values)

    error('sweep_values cannot be empty.');

end


sweep_values = sweep_values(:);

num_cases = length(sweep_values);


fprintf('\n');
fprintf('========================================\n');
fprintf('EGH446 FINAL PARAMETER SWEEP\n');
fprintf('========================================\n');
fprintf('Parameter: %s\n', sweep_parameter);
fprintf('Cases:     %d\n', num_cases);
fprintf('========================================\n');


%% ========================================================================
%  LOAD MODEL AND STORE ORIGINAL INITFCN
%  ========================================================================

load_system(modelName);

originalInitFcn = ...
    get_param(modelName, 'InitFcn');


%% ========================================================================
%  HIDE FIGURES DURING AUTOMATED TESTING
%  ========================================================================

oldFigureVisibility = ...
    get(groot, 'defaultFigureVisible');

set(groot, ...
    'defaultFigureVisible', ...
    'off');


%% ========================================================================
%  AUTOMATIC CLEANUP
%
%  Restores the original model InitFcn even if the script errors.
%  ========================================================================

cleanupObject = onCleanup(@() ...
    restoreSweepEnvironment( ...
        modelName, ...
        originalInitFcn, ...
        oldFigureVisibility));


%% ========================================================================
%  OBTAIN CURRENT FINAL SYSTEM CONFIGURATION
%
%  Run the normal model once so that the currently implemented final
%  controller/guidance parameters are available.
%  ========================================================================

fprintf('\nReading current final-system configuration...\n');

rng(fixed_rng_seed);

setupSimOut = sim( ...
    modelName, ...
    'ReturnWorkspaceOutputs', ...
    'on');


%% Retrieve Setup Logs

setupLogs = setupSimOut.get('logsout');


%% ========================================================================
%  READ CURRENT BASELINE PARAMETERS
%  ========================================================================

required_parameters = { ...
    'K_theta', ...
    'K_dist', ...
    'v_max', ...
    'omega_max', ...
    'heading_speed_min', ...
    'rvwp_lookahead', ...
    'capture_radius'};


for k = 1:length(required_parameters)

    variableName = required_parameters{k};

    if ~evalin( ...
            'base', ...
            sprintf('exist(''%s'',''var'')', variableName))

        error( ...
            ['Required parameter "%s" was not found in the base ' ...
             'workspace after model initialisation.'], ...
            variableName);

    end

end


baseline.K_theta = ...
    evalin('base', 'K_theta');

baseline.K_dist = ...
    evalin('base', 'K_dist');

baseline.v_max = ...
    evalin('base', 'v_max');

baseline.omega_max = ...
    evalin('base', 'omega_max');

baseline.heading_speed_min = ...
    evalin('base', 'heading_speed_min');

baseline.rvwp_lookahead = ...
    evalin('base', 'rvwp_lookahead');

baseline.capture_radius = ...
    evalin('base', 'capture_radius');


fprintf('\nCurrent baseline parameters\n');
fprintf('---------------------------\n');

fprintf( ...
    'K_theta             = %.6g\n', ...
    baseline.K_theta);

fprintf( ...
    'K_dist              = %.6g\n', ...
    baseline.K_dist);

fprintf( ...
    'v_max               = %.6g m/s\n', ...
    baseline.v_max);

fprintf( ...
    'omega_max           = %.6g rad/s\n', ...
    baseline.omega_max);

fprintf( ...
    'heading_speed_min   = %.6g\n', ...
    baseline.heading_speed_min);

fprintf( ...
    'rvwp_lookahead      = %.6g m\n', ...
    baseline.rvwp_lookahead);

fprintf( ...
    'capture_radius      = %.6g m\n', ...
    baseline.capture_radius);


%% ========================================================================
%  CREATE OR LOAD FIXED SWEEP MISSION
%  ========================================================================

if isfile(fixedMissionFile)

    fprintf('\nLoading existing fixed sweep mission:\n');
    fprintf('  %s\n', fixedMissionFile);

    fixedData = load(fixedMissionFile);

    fixed_robot = ...
        fixedData.fixed_robot;

    fixed_wp_ordered = ...
        fixedData.fixed_wp_ordered;

    fixed_wp_plot = ...
        fixedData.fixed_wp_plot;

else

    fprintf('\n');
    fprintf('No existing fixed sweep mission found.\n');
    fprintf('Creating one from the current final system...\n');


    %% Retrieve mission information generated by setup simulation

    if ~evalin('base', 'exist(''robot'',''var'')')

        error( ...
            'robot was not created by the model initialisation.');

    end


    if ~evalin('base', 'exist(''wp_ordered'',''var'')')

        error( ...
            'wp_ordered was not created by the model initialisation.');

    end


    if ~evalin('base', 'exist(''wp_plot'',''var'')')

        error( ...
            'wp_plot was not created by the model initialisation.');

    end


    fixed_robot = ...
        evalin('base', 'robot');

    fixed_wp_ordered = ...
        evalin('base', 'wp_ordered');

    fixed_wp_plot = ...
        evalin('base', 'wp_plot');


    %% Save permanently for every future sweep

    save( ...
        fixedMissionFile, ...
        'fixed_robot', ...
        'fixed_wp_ordered', ...
        'fixed_wp_plot');


    fprintf('\nFixed sweep mission saved to:\n');
    fprintf('  %s\n', fixedMissionFile);

end


%% ========================================================================
%  DISPLAY FIXED MISSION INFORMATION
%  ========================================================================

fprintf('\nFixed Mission\n');
fprintf('-------------\n');

fprintf( ...
    'Initial X             = %.3f m\n', ...
    fixed_robot.X);

fprintf( ...
    'Initial Y             = %.3f m\n', ...
    fixed_robot.Y);

fprintf( ...
    'Initial Heading       = %.3f deg\n', ...
    rad2deg(fixed_robot.Theta));

fprintf( ...
    'Number of Waypoints   = %d\n', ...
    size(fixed_wp_ordered,1));


%% ========================================================================
%  CREATE SWEEP-SPECIFIC INITFCN
%
%  Order is deliberately:
%
%    1. mdl_robot
%    2. controller_params
%    3. restore fixed mission
%    4. override controller/guidance parameters
%
%  This prevents controller_params from overwriting the swept value.
%  ========================================================================

sweepInitFcn = [ ...
    'mdl_robot;' ...
    'controller_params;' ...
    ...
    'robot = evalin(''base'',''SWEEP_robot'');' ...
    'wp_ordered = evalin(''base'',''SWEEP_wp_ordered'');' ...
    'wp_plot = evalin(''base'',''SWEEP_wp_plot'');' ...
    ...
    'K_theta = evalin(''base'',''SWEEP_K_theta'');' ...
    'K_dist = evalin(''base'',''SWEEP_K_dist'');' ...
    'v_max = evalin(''base'',''SWEEP_v_max'');' ...
    'omega_max = evalin(''base'',''SWEEP_omega_max'');' ...
    ...
    'heading_speed_min = ' ...
    'evalin(''base'',''SWEEP_heading_speed_min'');' ...
    ...
    'rvwp_lookahead = ' ...
    'evalin(''base'',''SWEEP_rvwp_lookahead'');' ...
    ...
    'capture_radius = ' ...
    'evalin(''base'',''SWEEP_capture_radius'');' ...
    ];


set_param( ...
    modelName, ...
    'InitFcn', ...
    sweepInitFcn);


%% ========================================================================
%  SEND FIXED MISSION TO BASE WORKSPACE
%  ========================================================================

assignin( ...
    'base', ...
    'SWEEP_robot', ...
    fixed_robot);

assignin( ...
    'base', ...
    'SWEEP_wp_ordered', ...
    fixed_wp_ordered);

assignin( ...
    'base', ...
    'SWEEP_wp_plot', ...
    fixed_wp_plot);


%% ========================================================================
%  PREALLOCATE PARAMETER VALUES
%  ========================================================================

Case = (1:num_cases).';

ParameterValue = sweep_values;


K_theta_used = ...
    nan(num_cases,1);

K_dist_used = ...
    nan(num_cases,1);

v_max_used = ...
    nan(num_cases,1);

omega_max_used = ...
    nan(num_cases,1);

heading_speed_min_used = ...
    nan(num_cases,1);

rvwp_lookahead_used = ...
    nan(num_cases,1);

capture_radius_used = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE MISSION PERFORMANCE
%  ========================================================================

WaypointsPlanned = ...
    nan(num_cases,1);

WaypointsCaptured = ...
    nan(num_cases,1);

MissionSuccess = ...
    false(num_cases,1);

MissionTime = ...
    nan(num_cases,1);

RouteLength = ...
    nan(num_cases,1);

TimePerMetre = ...
    nan(num_cases,1);

EffectiveMissionSpeed = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE CONTROLLER HEADING PERFORMANCE
%  ========================================================================

ControllerRMSHeading = ...
    nan(num_cases,1);

ControllerMeanAbsHeading = ...
    nan(num_cases,1);

ControllerMaxHeading = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE GROUND-TRUTH HEADING PERFORMANCE
%  ========================================================================

TrueRMSHeading = ...
    nan(num_cases,1);

TrueMeanAbsHeading = ...
    nan(num_cases,1);

TrueMaxHeading = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE WAYPOINT CAPTURE PERFORMANCE
%  ========================================================================

EstimatedCaptureError = ...
    nan(num_cases,1);

TrueCaptureError = ...
    nan(num_cases,1);

TrueMaxCaptureError = ...
    nan(num_cases,1);

TrueMinCaptureError = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE GROUND-TRUTH CROSS-TRACK PERFORMANCE
%  ========================================================================

TrueRMSCrossTrack = ...
    nan(num_cases,1);

TrueMeanCrossTrack = ...
    nan(num_cases,1);

TrueMaxCrossTrack = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE CONTROLLER COMMAND PERFORMANCE
%  ========================================================================

MeanVcmd = ...
    nan(num_cases,1);

MaxVcmd = ...
    nan(num_cases,1);

MeanAbsOmega = ...
    nan(num_cases,1);

MaxAbsOmega = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE ESTIMATOR PERFORMANCE
%  ========================================================================

EstimatorXRMSE = ...
    nan(num_cases,1);

EstimatorYRMSE = ...
    nan(num_cases,1);

EstimatorPositionRMSE = ...
    nan(num_cases,1);

EstimatorHeadingRMSE = ...
    nan(num_cases,1);


%% ========================================================================
%  PREALLOCATE YAW SATURATION PERFORMANCE
%  ========================================================================

YawSaturationTime = ...
    nan(num_cases,1);

YawSaturationPercent = ...
    nan(num_cases,1);


%% ========================================================================
%  RUN CONTROLLED PARAMETER SWEEP
%  ========================================================================

for case_index = 1:num_cases

    fprintf('\n');
    fprintf('========================================\n');
    fprintf('SWEEP CASE %d OF %d\n', ...
        case_index, ...
        num_cases);
    fprintf('========================================\n');

    fprintf( ...
        '%s = %.6g\n', ...
        sweep_parameter, ...
        sweep_values(case_index));


    %% --------------------------------------------------------------------
    % Start every case from current baseline parameter set
    % ---------------------------------------------------------------------

    test = baseline;


    %% --------------------------------------------------------------------
    % Change ONLY selected parameter
    % ---------------------------------------------------------------------

    test.(sweep_parameter) = ...
        sweep_values(case_index);


    %% --------------------------------------------------------------------
    % Store exact parameter values for traceability
    % ---------------------------------------------------------------------

    K_theta_used(case_index) = ...
        test.K_theta;

    K_dist_used(case_index) = ...
        test.K_dist;

    v_max_used(case_index) = ...
        test.v_max;

    omega_max_used(case_index) = ...
        test.omega_max;

    heading_speed_min_used(case_index) = ...
        test.heading_speed_min;

    rvwp_lookahead_used(case_index) = ...
        test.rvwp_lookahead;

    capture_radius_used(case_index) = ...
        test.capture_radius;


    %% --------------------------------------------------------------------
    % Send parameters to base workspace
    % ---------------------------------------------------------------------

    assignin( ...
        'base', ...
        'SWEEP_K_theta', ...
        test.K_theta);

    assignin( ...
        'base', ...
        'SWEEP_K_dist', ...
        test.K_dist);

    assignin( ...
        'base', ...
        'SWEEP_v_max', ...
        test.v_max);

    assignin( ...
        'base', ...
        'SWEEP_omega_max', ...
        test.omega_max);

    assignin( ...
        'base', ...
        'SWEEP_heading_speed_min', ...
        test.heading_speed_min);

    assignin( ...
        'base', ...
        'SWEEP_rvwp_lookahead', ...
        test.rvwp_lookahead);

    assignin( ...
        'base', ...
        'SWEEP_capture_radius', ...
        test.capture_radius);


    %% --------------------------------------------------------------------
    % Reset MATLAB random seed
    % ---------------------------------------------------------------------

    rng(fixed_rng_seed);


    %% --------------------------------------------------------------------
    % Run simulation
    % ---------------------------------------------------------------------

    try

        simOut = sim( ...
            modelName, ...
            'ReturnWorkspaceOutputs', ...
            'on');


        %% Retrieve logged signals

        logsout = ...
            simOut.get('logsout');


        %% ---------------------------------------------------------------
        % Run final controller analysis
        % ---------------------------------------------------------------

        if show_detailed_run_output

            controller_analysis;

        else

            evalc('controller_analysis;');

        end


        %% ---------------------------------------------------------------
        % Mission results
        % ---------------------------------------------------------------

        WaypointsPlanned(case_index) = ...
            size(fixed_wp_ordered,1);

        WaypointsCaptured(case_index) = ...
            num_true_waypoints_captured;

        MissionSuccess(case_index) = ...
            num_true_waypoints_captured == ...
            WaypointsPlanned(case_index);


        MissionTime(case_index) = ...
            mission_time;

        RouteLength(case_index) = ...
            route_length;

        TimePerMetre(case_index) = ...
            mission_time_per_metre;

        EffectiveMissionSpeed(case_index) = ...
            mission_effective_speed;


        %% ---------------------------------------------------------------
        % Controller heading results
        % ---------------------------------------------------------------

        ControllerRMSHeading(case_index) = ...
            heading_error_rms;

        ControllerMeanAbsHeading(case_index) = ...
            heading_error_mean_abs;

        ControllerMaxHeading(case_index) = ...
            heading_error_max_abs;


        %% ---------------------------------------------------------------
        % Ground-truth heading results
        % ---------------------------------------------------------------

        TrueRMSHeading(case_index) = ...
            true_heading_error_rms;

        TrueMeanAbsHeading(case_index) = ...
            true_heading_error_mean_abs;

        TrueMaxHeading(case_index) = ...
            true_heading_error_max_abs;


        %% ---------------------------------------------------------------
        % Capture results
        % ---------------------------------------------------------------

        EstimatedCaptureError(case_index) = ...
            mean_capture_error;

        TrueCaptureError(case_index) = ...
            mean_true_capture_error;

        TrueMaxCaptureError(case_index) = ...
            max_true_capture_error;

        TrueMinCaptureError(case_index) = ...
            min_true_capture_error;


        %% ---------------------------------------------------------------
        % Ground-truth finite-segment cross-track results
        % ---------------------------------------------------------------

        TrueRMSCrossTrack(case_index) = ...
            rms_cross_track_error;

        TrueMeanCrossTrack(case_index) = ...
            mean_cross_track_error;

        TrueMaxCrossTrack(case_index) = ...
            max_cross_track_error;


        %% ---------------------------------------------------------------
        % Controller commands
        % ---------------------------------------------------------------

        MeanVcmd(case_index) = ...
            mean_v_cmd;

        MaxVcmd(case_index) = ...
            max_v_cmd;

        MeanAbsOmega(case_index) = ...
            mean_abs_omega_cmd;

        MaxAbsOmega(case_index) = ...
            max_omega_cmd;


        %% ---------------------------------------------------------------
        % Estimator performance
        % ---------------------------------------------------------------

        EstimatorXRMSE(case_index) = ...
            estimator_rmse_x;

        EstimatorYRMSE(case_index) = ...
            estimator_rmse_y;

        EstimatorPositionRMSE(case_index) = ...
            estimator_rmse_position;

        EstimatorHeadingRMSE(case_index) = ...
            estimator_rmse_heading;


        %% ---------------------------------------------------------------
        % Yaw saturation performance
        % ---------------------------------------------------------------

        YawSaturationTime(case_index) = ...
            yaw_saturated_time;

        YawSaturationPercent(case_index) = ...
            yaw_saturation_percent;


        %% ---------------------------------------------------------------
        % Short run summary
        % ---------------------------------------------------------------

        fprintf( ...
            'Mission Time             = %.3f s\n', ...
            MissionTime(case_index));

        fprintf( ...
            'TRUE RMS Heading         = %.3f deg\n', ...
            TrueRMSHeading(case_index));

        fprintf( ...
            'TRUE RMS Cross-Track     = %.3f m\n', ...
            TrueRMSCrossTrack(case_index));

        fprintf( ...
            'TRUE Capture Error       = %.3f m\n', ...
            TrueCaptureError(case_index));

        fprintf( ...
            'Mean v_cmd               = %.3f m/s\n', ...
            MeanVcmd(case_index));

        fprintf( ...
            'Mean |omega_cmd|         = %.3f rad/s\n', ...
            MeanAbsOmega(case_index));

        fprintf( ...
            'Yaw Saturation           = %.2f %%\n', ...
            YawSaturationPercent(case_index));

        fprintf( ...
            'Waypoints Captured       = %d/%d\n', ...
            WaypointsCaptured(case_index), ...
            WaypointsPlanned(case_index));


    catch ME

        warning( ...
            'Sweep case %d failed for %s = %.6g.', ...
            case_index, ...
            sweep_parameter, ...
            sweep_values(case_index));

        fprintf('\nSimulation error:\n%s\n', ME.message);

        MissionSuccess(case_index) = false;

    end

end


%% ========================================================================
%  CREATE COMPLETE SWEEP RESULTS TABLE
%  ========================================================================

results = table( ...
    Case, ...
    ParameterValue, ...
    K_theta_used, ...
    K_dist_used, ...
    v_max_used, ...
    omega_max_used, ...
    heading_speed_min_used, ...
    rvwp_lookahead_used, ...
    capture_radius_used, ...
    WaypointsPlanned, ...
    WaypointsCaptured, ...
    MissionSuccess, ...
    MissionTime, ...
    RouteLength, ...
    TimePerMetre, ...
    EffectiveMissionSpeed, ...
    ControllerRMSHeading, ...
    ControllerMeanAbsHeading, ...
    ControllerMaxHeading, ...
    TrueRMSHeading, ...
    TrueMeanAbsHeading, ...
    TrueMaxHeading, ...
    EstimatedCaptureError, ...
    TrueCaptureError, ...
    TrueMaxCaptureError, ...
    TrueMinCaptureError, ...
    TrueRMSCrossTrack, ...
    TrueMeanCrossTrack, ...
    TrueMaxCrossTrack, ...
    MeanVcmd, ...
    MaxVcmd, ...
    MeanAbsOmega, ...
    MaxAbsOmega, ...
    EstimatorXRMSE, ...
    EstimatorYRMSE, ...
    EstimatorPositionRMSE, ...
    EstimatorHeadingRMSE, ...
    YawSaturationTime, ...
    YawSaturationPercent);


%% ========================================================================
%  DISPLAY COMPLETE RESULTS
%  ========================================================================

fprintf('\n\n');
fprintf('========================================\n');
fprintf('FINAL %s SWEEP RESULTS\n', upper(sweep_parameter));
fprintf('========================================\n\n');

disp(results);


%% ========================================================================
%  CREATE REPORT-FOCUSED TABLE
%
%  Contains the metrics most likely to be used directly in the report.
%  ========================================================================

report_results = table( ...
    ParameterValue, ...
    MissionTime, ...
    TrueRMSHeading, ...
    TrueMeanAbsHeading, ...
    TrueMaxHeading, ...
    TrueCaptureError, ...
    TrueRMSCrossTrack, ...
    TrueMeanCrossTrack, ...
    TrueMaxCrossTrack, ...
    MeanVcmd, ...
    MaxVcmd, ...
    MeanAbsOmega, ...
    MaxAbsOmega, ...
    YawSaturationPercent, ...
    WaypointsCaptured, ...
    MissionSuccess);


fprintf('\n');
fprintf('========================================\n');
fprintf('REPORT-READY SWEEP TABLE\n');
fprintf('========================================\n\n');

disp(report_results);


%% ========================================================================
%  AUTOMATIC OUTPUT FILE NAMES
%  ========================================================================

safe_parameter_name = ...
    matlab.lang.makeValidName(sweep_parameter);

csvFile = ...
    sprintf( ...
        'sweep_%s_results.csv', ...
        safe_parameter_name);

reportCsvFile = ...
    sprintf( ...
        'sweep_%s_report.csv', ...
        safe_parameter_name);

excelFile = ...
    sprintf( ...
        'sweep_%s_results.xlsx', ...
        safe_parameter_name);

matFile = ...
    sprintf( ...
        'sweep_%s_results.mat', ...
        safe_parameter_name);


%% ========================================================================
%  WRITE COMPLETE RESULTS
%  ========================================================================

writetable( ...
    results, ...
    csvFile);

writetable( ...
    report_results, ...
    reportCsvFile);


%% ========================================================================
%  WRITE EXCEL WORKBOOK
%
%  Sheet 1 = complete analysis output
%  Sheet 2 = compact report-ready output
%  ========================================================================

writetable( ...
    results, ...
    excelFile, ...
    'Sheet', ...
    'Complete Results');

writetable( ...
    report_results, ...
    excelFile, ...
    'Sheet', ...
    'Report Results');


%% ========================================================================
%  SAVE MAT FILE
%  ========================================================================

save( ...
    matFile, ...
    'results', ...
    'report_results', ...
    'baseline', ...
    'sweep_parameter', ...
    'sweep_values', ...
    'fixed_robot', ...
    'fixed_wp_ordered', ...
    'fixed_wp_plot');


%% ========================================================================
%  RESTORE ORIGINAL MODEL INITFCN
%  ========================================================================

set_param( ...
    modelName, ...
    'InitFcn', ...
    originalInitFcn);


%% ========================================================================
%  RESTORE FIGURES
%  ========================================================================

set(groot, ...
    'defaultFigureVisible', ...
    oldFigureVisibility);

close all;


%% ========================================================================
%  CLEAN SWEEP VARIABLES FROM BASE WORKSPACE
%  ========================================================================

sweep_base_variables = { ...
    'SWEEP_robot', ...
    'SWEEP_wp_ordered', ...
    'SWEEP_wp_plot', ...
    'SWEEP_K_theta', ...
    'SWEEP_K_dist', ...
    'SWEEP_v_max', ...
    'SWEEP_omega_max', ...
    'SWEEP_heading_speed_min', ...
    'SWEEP_rvwp_lookahead', ...
    'SWEEP_capture_radius'};


for k = 1:length(sweep_base_variables)

    variableName = ...
        sweep_base_variables{k};

    if evalin( ...
            'base', ...
            sprintf('exist(''%s'',''var'')', variableName))

        evalin( ...
            'base', ...
            sprintf('clear %s', variableName));

    end

end


%% ========================================================================
%  COMPLETION MESSAGE
%  ========================================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('PARAMETER SWEEP COMPLETE\n');
fprintf('========================================\n');

fprintf( ...
    'Parameter tested: %s\n', ...
    sweep_parameter);

fprintf( ...
    'Fixed mission:    %s\n', ...
    fixedMissionFile);

fprintf('\nFiles created:\n');

fprintf( ...
    '  %s\n', ...
    csvFile);

fprintf( ...
    '  %s\n', ...
    reportCsvFile);

fprintf( ...
    '  %s\n', ...
    excelFile);

fprintf( ...
    '  %s\n', ...
    matFile);

fprintf('\n');
fprintf('Use the "Report Results" Excel sheet for final report tables.\n');
fprintf('========================================\n');


%% ========================================================================
%  LOCAL CLEANUP FUNCTION
%  ========================================================================

function restoreSweepEnvironment( ...
    modelName, ...
    originalInitFcn, ...
    oldFigureVisibility)

    % Restore original model initialisation callback
    try

        if bdIsLoaded(modelName)

            set_param( ...
                modelName, ...
                'InitFcn', ...
                originalInitFcn);

        end

    catch

    end


    % Restore MATLAB figure visibility
    try

        set(groot, ...
            'defaultFigureVisible', ...
            oldFigureVisibility);

    catch

    end


    % Remove temporary sweep variables from base workspace
    sweepVariables = { ...
        'SWEEP_robot', ...
        'SWEEP_wp_ordered', ...
        'SWEEP_wp_plot', ...
        'SWEEP_K_theta', ...
        'SWEEP_K_dist', ...
        'SWEEP_v_max', ...
        'SWEEP_omega_max', ...
        'SWEEP_heading_speed_min', ...
        'SWEEP_rvwp_lookahead', ...
        'SWEEP_capture_radius'};


    for n = 1:length(sweepVariables)

        variableName = ...
            sweepVariables{n};

        try

            if evalin( ...
                    'base', ...
                    sprintf( ...
                    'exist(''%s'',''var'')', ...
                    variableName))

                evalin( ...
                    'base', ...
                    sprintf( ...
                    'clear %s', ...
                    variableName));

            end

        catch

        end

    end

end