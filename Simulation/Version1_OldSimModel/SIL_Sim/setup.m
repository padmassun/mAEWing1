%% setup.m
% UAV Software-in-the-Loop Simulation setup
%
% This script will setup the SIL simulation. Stored aircraft configuration
% and trim conditions are used.
%
% University of Minnesota
% Aerospace Engineering and Mechanics
% Copyright 2011 Regents of the University of Minnesota.
% All rights reserved.
%

%% Clean up
clearvars
close all
bdclose all
clc

%% Add Libraries and controllers folder to MATLAB path
addpath ../Libraries
addpath ../Controllers
warning off Simulink:Engine:SaveWithParameterizedLinks_Warning
warning off Simulink:Commands:LoadMdlParameterizedLink
warning off Simulink:ID:DuplicateSID

%% Load airframe configuration and trim condition
% To change these, use the functions "UAV_config.m" and "trim_UAV.m"
load UAV_modelconfig
load UAV_trimcondition

%% Simulation sample time
SampleTime = 0.01; % sec

%% Set controller variants
% Each variant corresponds to a different Simulink model that will be
% referenced in the "UAV_SIL/Control Software/Control Software" block.
flightcode_var = Simulink.Variant('controller_mode == 1 || controller_mode == 3 || controller_mode == 5 || controller_mode == 7 || controller_mode == 9');
baseline_control_var = Simulink.Variant('controller_mode == 2');
autoland_control_var = Simulink.Variant('controller_mode == 4');
%% Set controller mode
% Use this variable to quickly change what controller is used in the
% simulation.
%
% 1 = baseline controller (C implementation)
% 2 = baseline controller (Simulink)
% 4 = autoland controller for minimutt (Simulink)
controller_mode = 1;

% Load controller parameters or compile flight code
switch controller_mode
    case 1 % Flight Controller in C
        % Compile Flight Software:

    case 2 % Baseline controller in Simulink.
        roll_gains = [0.5, 0.15, 0.00];        %
        pitch_gains = [-0.3, -0.35, -0.01];
    case 4 % Auto-Land controller in Simulink.        
        roll_gains = [0.5, 0.15, 0.00];        %
        % pitch_gains = [-0.3, -0.4, -0.01];   % with elevon mixing
        pitch_gains = [-0.75, -1.0, -0.01];   % with elevon mixing
        autothrottle_gains = [0.1, 0.02];  %
        sinkrate_gains  = [-0.01,  -0.05]; % 
       
        approach_theta_sat = [-20 20];   % [deg]
        landing_theta_sat = [-10 10];   % [deg]
        engage_flare = 40; % [m]
        
        approach_sinkrate = 3; %[m/s]
        approach_speed = 20; % [m/s]
        landing_sinkrate = 0.5;  % [m/s]
        landing_speed = 16; % [m/s]
        
        autoland_flag = true;
end

% Advanced Users: Include guidance, system ID, or fault injection codes.
% Set the path to the .c code in the following variables.
%


%%%%% GUIDANCE LAW %%%%%
% Point to the desired guidance code here. Use '-DSIMULINK_GUIDANCE' to
% input the reference commands from the simulink diagram.
% GUIDANCE = '../../FlightCode/guidance/doublet_phi_theta.c';
% GUIDANCE = '../../FlightCode/guidance/straight_level.c';
% GUIDANCE = '../../FlightCode/guidance/empty_guidance.c';
GUIDANCE = '-DSIMULINK_GUIDANCE';

%%%%% CONTROL LAW %%%%%
% CONTROL =  '../../Software/FlightCode/control/tres_claw.c';
% CONTROL =  '../../Software/FlightCode/control/control_law_for_flutter_suppression.c ../../Software/FlightCode/control/ss_flutter_suppression01.c';
CONTROL =  '../../Software/FlightCode/control/control_law_for_sysid.c';

%%%%%% SYSTEM ID SELECTION %%%%%
% Point to the desired system ID code here
% SYSTEM_ID = '../../Software/FlightCode/system_id/systemid_none.c';
% SYSTEM_ID = '../../Software/FlightCode/system_id/tres_sysid.c';
SYSTEM_ID = '../../Software/FlightCode/system_id/sysid_pitch_chirps.c';

%%%%%% SURFACE FAULT MODE SELECTION %%%%%
% Point to the desired fault code here
SURFACE_FAULT = '../../Software/FlightCode/faults/surffault_none.c';

%%%%%% SENS0R FAULT MODE SELECTION %%%%%
% Point to the desired fault code here
SENSOR_FAULT = '../../Software/FlightCode/faults/sensfault_none.c';

% Compile control software
if exist('CONTROL','var')  % XXX check all conditions
    eval(['mex -I../../Software/FlightCode/ control_SIL.c '...   
        GUIDANCE ' '  SYSTEM_ID ' ' CONTROL ' ' ... 
        '-DAIRCRAFT_UP1DIR=\"../aircraft/skoll_config.h\"' ...
        ' ../../Software/FlightCode/utils/matrix_SIL.c ' ...
        ' ../../Software/FlightCode/control/control_functions.c ' ...
        ' ../../Software/FlightCode/system_id/systemid_functions.c ']);
end

%% Integer Time delay in flight software loop
IntegerTimeDelay = 2; % .04sec

%% Open sim diagram
UAV_SIL
