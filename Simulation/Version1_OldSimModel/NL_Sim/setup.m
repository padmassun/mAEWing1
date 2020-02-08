%% setup.m
% UAV Nonlinear Simulation setup
%
% This script will setup the nonlinear simulation (UAV_NL.mdl) and call
% trim and linearization routines. Select the desired aircraft here in this
% script, via the "UAV_config()" function call.
%
% Note: the UAV_NL.mdl model is not opened by default. This is not
% necessary to trim, linearize, and simulate via command line inputs.
%
% Calls: UAV_config.m
%        trim_UAV.m
%        linearize_UAV.m
%
% University of Minnesota
% Aerospace Engineering and Mechanics
% Copyright 2011 Regents of the University of Minnesota.
% All rights reserved.
%

% clean up
clear all
close all
bdclose all
clc

%% Add Libraries folder to MATLAB path
addpath ../Libraries
warning off Simulink:Engine:SaveWithParameterizedLinks_Warning
warning off Simulink:Commands:LoadMdlParameterizedLink
warning off Simulink:ID:DuplicateSID

%% Configure Airframe, either 'UltraStick120', 'UltraStick25e', or 'miniMUTT_rigid'
% [AC,Env] = UAV_config('UltraStick120');
% [AC,Env] = UAV_config('UltraStick25e');
[AC,Env] = UAV_config('minimutt_rigid');


%% Simulation sample time
SampleTime = 0.02; % sec

%% Set aircraft initial conditions
% Note: these are NOT the trim condition targets. If the trim fails to
% coverge, try using different initial conditions. Also note that the trim
% function will overwrite these initial conditions with the trimmed
% conditions.

% Set initial state values
TrimCondition.InertialIni    = [0 0 -100]';   % Initial Position in Inertial Frame [Xe Ye Ze], [m]
TrimCondition.LLIni          = [44.7258357 -93.07501316]';     % Initial Latitude/Longitude of Aircraft [Lat Long], [deg]
TrimCondition.AttitudeIni    = [0 0.0217 155*pi/180]'; % Initial Euler orientation [roll,pitch,yaw] [rad], can't use 0 heading, causes large entry in C matrix for psi
TrimCondition.RatesIni       = [0 0 0]';      % Initial Body Frame rotation rates [p q r], [rad/s]

% Steady state wind values, m/s
TrimCondition.Inputs.Wind     = [0 0 0]';

switch lower(AC.aircraft)
    
    case 'ultrastick120'
        % Control surface initial values, rad:
        % elevator, rudder, aileron, left flap, right flap
        TrimCondition.Inputs.CtrlSurf = [0.091 0 0 0 0]';
        % number of the control surfaces (required to initialize the input
        % port dimension; dynamically relocating it returns an error)
        AC.nctrls = size(TrimCondition.Inputs.CtrlSurf,1);
        
        % Throttle initial value, nd
        TrimCondition.Inputs.Throttle = 0.559;
        
        % Initial velocity vector, m/s: [u v w]
        TrimCondition.VelocitiesIni  = [23 0 0.369]';
        
        % Initial Engine Speed [rad/s]
        TrimCondition.EngineSpeedIni = 827;
        
    case 'ultrastick25e'
        % Control surface initial values, rad:
        % elevator, rudder, aileron, left flap, right flap
        TrimCondition.Inputs.CtrlSurf = [0.091 0 0 0 0]';
        
        % number of the control surfaces (required to initialize the input
        % port dimension; dynamically relocating it returns an error)
        AC.nctrls = size(TrimCondition.Inputs.CtrlSurf,1);
        
        % Throttle initial value, nd
        TrimCondition.Inputs.Throttle = 0.559;
        
        % Initial velocity vector, m/s: [u v w]
        TrimCondition.VelocitiesIni  = [17 0 0.369]';
        
        % Initial Engine Speed [rad/s]
        TrimCondition.EngineSpeedIni = 827;
        
        
    case 'minimutt_rigid'
        % Control surface initial values, rad:
        % elevator, L1, aileron, R1, L4, R4
        TrimCondition.Inputs.CtrlSurf = [0 0 0 0 -.07 -.07 -.07 -.07]';
        AC.nctrls = size(TrimCondition.Inputs.CtrlSurf,1);
        
        % Throttle initial value, nd
        TrimCondition.Inputs.Throttle = 0.559;
        
        % Initial velocity vector, m/s: [u v w]
        TrimCondition.VelocitiesIni  = [23 0 0.369]';
        
        % Initial Engine Speed [rad/s]
        TrimCondition.EngineSpeedIni = 827;
end


%% Trim aircraft to a specific flight condition
% Set the trim targets here. See trim_UAV for complete list of target
% variables. Sideslip angle (beta), flight path angle (gamma) and flap
% setting default to zero. If a fixed control input setting is desired,
% specify as a target.

% straight and level, (m/s, rad)
TrimCondition.target = struct('V_s',TrimCondition.VelocitiesIni(1),'gamma',0,'h',100);
%TrimCondition.target = struct('V_s',15,'gamma',0,'h',350,'L1',0*pi/180,'L2',-0*pi/180,'L4',0*pi/180,'R1',0*pi/180,'R2',-0*pi/180,'R4',0*pi/180); %level flight
% TrimCondition.target = struct('V_s',TrimCondition.VelocitiesIni(1),'gamma',5/180*pi); % level climb, (m/s, rad)
% TrimCondition.target = struct('V_s',TrimCondition.VelocitiesIni(1),'gamma',0,'psidot',20/180*pi); % level turn, (m/s, rad, rad/sec)
% TrimCondition.target = struct('V_s',TrimCondition.VelocitiesIni(1),'gamma',5/180*pi,'psidot',20/180*pi); % climbing turn, (m/s, rad, rad/sec)
% TrimCondition.target = struct('V_s',TrimCondition.VelocitiesIni(1),'gamma',0,'beta',5/180*pi); % level steady heading sideslip, (m/s, rad, rad)

% Find the trim solution
[TrimCondition,OperatingPoint] = trim_UAV(TrimCondition,AC);

%% Linearize about the operating point
[longmod,spmod,latmod,linmodel]=linearize_UAV(OperatingPoint,AC);
