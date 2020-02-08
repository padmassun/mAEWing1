function [AC] = miniMUTT_rigid_config()
% function [AC] = miniMUTT_rigid_config()
% MiniMUTT (Rigid) configuration file. Sets aircraft parameters.
% Called from: UAV_config.m
%
% University of Minnesota 
% Aerospace Engineering and Mechanics 
% Copyright 2011 Regents of the University of Minnesota. All rights reserved.
%
% SVN Info: $Id: Ultrastick_config.m 765 2012-01-25 20:14:05Z murch $

% AC.Geometry, let's use X positive forward, y right wing, z down...
% reference point is the nose

%% Aircraft and type
AC.aircraft = 'minimutt_rigid';
AC.type = ('');

%% Inertia, Mass and CG location
% Gross aircraft mass [kg] (including relevant hardware)
AC.Mass  = 7.37; % BRT XX Fenrir weight

% CG location [x y z], [m]
% PS 05/03/2015 (taken from website, word of mouth from AK)
AC.Geometry.rCG = [-0.59 0 0]; 

% BRT from CR testing 2015-06-05
% Gross moments of inertia [Jx Jy Jz Jxz] [kg*m^2]
AC.Inertia.Ixx = 3.098; %   
AC.Inertia.Iyy = 0.464; %  
AC.Inertia.Izz = 3.315; %   
AC.Inertia.Matrix = [AC.Inertia.Ixx     0               0;...
                        0           AC.Inertia.Iyy      0;...
                        0               0               AC.Inertia.Izz];

%% Aircraft Geometric Parameters

% Aerodynamic force application point (usually the aerodynamic center)[x y z]
% TO BE CHECKED PS 05/03/2015 (word of mouth from AK)
AC.Geometry.AeroCenter = [-0.58 0 0]; 
% Mean aerodynamic chord [m]
AC.Geometry.c = 0.3210;
% Wing span [m]
AC.Geometry.b = 3.0480;
% Wing area [m^2]
AC.Geometry.S = 1.05;


%% Aero coefficients
% Remarks:
% ALL stability derivative units are radians:

%% Linear Derivative Model
% Force coefficients are in WIND axes.
% See Klein, Morelli, Aircraft System Identification, pg 41

%% Lift coefficient
% Taken from mat-file provided by AK on 05/04/2015
% Zero-alpha lift
AC.Aero.CL.zero = 0.0; % Symmetric airfoil
% alpha derivative
AC.Aero.CL.alpha = 4.5598; 
% pitch-rate derivative
AC.Aero.CL.q = 4.1486; 
% Lift coefficients for control surfaces
AC.Aero.CL.L1 = 0.3731; AC.Aero.CL.R1 = 0.3731;
AC.Aero.CL.L2 = 0.3497; AC.Aero.CL.R2 = 0.3497;
AC.Aero.CL.L3 = 0.3124; AC.Aero.CL.R3 = 0.3124;
AC.Aero.CL.L4 = 0.2457; AC.Aero.CL.R4 = 0.2457;

%% Drag coefficient
% Taken from PAAW working paper by DKS
% % Lift at minimum drag
% AC.Aero.CL.minD = 0.0;
% Minimum drag
AC.Aero.CDw.min = 0.0;
% % Zero-alpha drag
% AC.Aero.CDw.zero = 0.0; % No data
% alpha derivative
AC.Aero.CDw.alpha = 0.129;
% % Oswald's coefficient
% AC.Aero.CDw.osw = 0.75; % Copied from UltraStick
% Drag coefficients for control surfaces
% To check if values reported in DKS's paper are for single surface of
% L-R pair deployed together.
AC.Aero.CDw.L1 = 0.0012; AC.Aero.CDw.R1 = 0.0012;
AC.Aero.CDw.L2 = 0.0015; AC.Aero.CDw.R2 = 0.0015;
AC.Aero.CDw.L3 = 0.0018; AC.Aero.CDw.R3 = 0.0018;
AC.Aero.CDw.L4 = 0.0012; AC.Aero.CDw.R4 = 0.0012;

%% Side Force coefficient 
% To be updated later
% beta derivative
AC.Aero.CYw.beta = -0.21;
% Roll rate derivative
AC.Aero.CYw.p = 0.0; 
% Yaw rate derivative
AC.Aero.CYw.r = 0.0515;
% Side-force coefficients for control surfaces
AC.Aero.CYw.L1 = 0.0; AC.Aero.CYw.R1 = 0.0;
AC.Aero.CYw.L2 = 0.0; AC.Aero.CYw.R2 = 0.0;
AC.Aero.CYw.L3 = 0.0; AC.Aero.CYw.R3 = 0.0;
AC.Aero.CYw.L4 = 0.0; AC.Aero.CYw.R4 = 0.0;

%% Roll moment coefficient
% Taken from mat-file provided by AK on 05/04/2015
% Sideslip derivative
AC.Aero.Cl.beta = 7.1802e-04;%7.1802e-04
% Roll rate derivative
AC.Aero.Cl.p = -0.5486; 
% Yaw rate derivative
AC.Aero.Cl.r = 0.0035; % No data
% Control derivatives
AC.Aero.Cl.L1 = 0.0200; AC.Aero.Cl.R1 = -0.0200;
AC.Aero.Cl.L2 = 0.0550; AC.Aero.Cl.R2 = -0.0550;
AC.Aero.Cl.L3 = 0.0853; AC.Aero.Cl.R3 = -0.0853;
AC.Aero.Cl.L4 = 0.0924; AC.Aero.Cl.R4 = -0.0924;

%% Pitch moment coefficient
% Taken from mat-file provided by AK on 05/04/2015
% Zero-alpha pitch
AC.Aero.Cm.zero = 0.0;
% alpha derivative
AC.Aero.Cm.alpha = -0.3299;
% pitch-rate derivative
AC.Aero.Cm.q = -2.0543; 
% Moment coefficients for control surfaces
AC.Aero.Cm.L1 =-8.7029e-04; AC.Aero.Cm.R1 = -8.7029e-04;
AC.Aero.Cm.L2 = -0.0329;    AC.Aero.Cm.R2 = -0.0329;
AC.Aero.Cm.L3 = -0.1301;    AC.Aero.Cm.R3 = -0.1301;
AC.Aero.Cm.L4 = -0.1870;    AC.Aero.Cm.R4 = -0.1870;

%% Yaw moment coefficient
% TO BE UPDATED
% Sideslip derivative
AC.Aero.Cn.beta = 0.03;
% Yaw rate derivative
AC.Aero.Cn.r = -0.0577; % BRT estimated from vertical volume coefficient
% Roll-rate derivative
AC.Aero.Cn.p = 0.0; 
% Moment coefficients for control surfaces
AC.Aero.Cn.L1 = 0.0;    AC.Aero.Cn.R1 = 0.0;
AC.Aero.Cn.L2 = 0.0;    AC.Aero.Cn.R2 = 0.0;
AC.Aero.Cn.L3 = 0.0;    AC.Aero.Cn.R3 = 0.0;
AC.Aero.Cn.L4 = 0.0;    AC.Aero.Cn.R4 = 0.0;

%% PROPULSION
% Taken from UltraStick120 with modified paramters as per Chris's email dated
% 05/20/2015
% Propulsion force application point (usually propeller hub) [x y z]
% For UltraStick120 coordinates are negative for propeller placed in front
% of reference point
% CR: Propulsion is applied at -32.99, 0, -1.24 inches
AC.Geometry.rProp = [-32.99*0.0254 0 -1.24*0.0254]; % [m]

% Thrust alignment orientation, radians
% CR: The thrust line is pointed down 1.64 degrees
AC.Prop.Angles = [0 1.64 0]/180*pi;

% Using an APC 12 x 6E propeller, EFlite Power 25 motor
% Coefficient of thrust polynomial model 
% AEM wind tunnel test data Jan 2011
AC.Prop.CT = [-0.4314 1.08 -0.896 0.1089 0.0604 ];

% Coefficient of power polynomial model 
% AEM wind tunnel test data Jan 2011
AC.Prop.CP = [0.5054 -0.5304 0.0412 0.01664 0.0223];

% Motor polynomial model. Input throttle, output power in watts
% EFlite Power 25, AEM wind tunnel test data Jan 2011
% Castle Creations ICE Lite 50 used to collect motor data
% Multiplied by 6 to get peak power of 1440 watts.
AC.Prop.Power = [174.46 70.135 -4.39]*6;

% Electric motor and propeller combine moment of inertia
% CR: Use the same as the what we have for the Ultrastick 120: 0.00041935.
AC.Prop.Jmp = 0.00041935;  % [kg*m^2] estimated, AMM 9/12/11

% Propeller diameter (m) 
% CR: 12" = 30.48 cm (diameter) -->  15.24cm radius
AC.Prop.Radius = 0.1524;

% CR: The units are Watts. The E-Flite Power 52 motor is rated to 1650 Watts.
AC.Prop.ThrottleOutputLimit.Upper = 1650;
AC.Prop.ThrottleOutputLimit.Lower = 0;
% CR: RPM limits should be the same.
AC.Prop.OmegaSaturation.Upper = 15000*pi/30;
AC.Prop.OmegaSaturation.Lower = 1;

%% Configure Actuators and Initial Conditions
% 6/2/2015 PJS: Second-order actuator model for Futaba S9254 based
% on report/modeling by I. Lakshminarayan and P. Seiler
%
% 6/4/2015 PJS: Positions limits currently set to +/-25degs. This
% might need to be updated for Skoll.

% No mini-MUTT servos data available yet
% Bandwidth data taken from BFF servos
% Rates limits not accurate
d2r = pi/180;                         % Degrees to Radians conversion

% Flutter suppression L1
%AC.Actuator.L1.BW = 70;            % [Hz]
AC.Actuator.L1.wn = 97.17;          % [rad/sec]
AC.Actuator.L1.zeta = 0.944;        % [rad/sec]
AC.Actuator.L1.RateLim = 1000*d2r;  % [rad/s]
AC.Actuator.L1.PosLim = 25*d2r;     % [rad]
AC.Actuator.L1.NegLim = -25*d2r;    %[rad]

% Left Aileron L2
%AC.Actuator.L2.BW = 30;            % [Hz]
AC.Actuator.L2.wn = 97.17;          % [rad/sec]
AC.Actuator.L2.zeta = 0.944;        % [rad/sec]
AC.Actuator.L2.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.L2.PosLim = 25*d2r;   % [rad]
AC.Actuator.L2.NegLim = -25*d2r;  %[rad]

% Elevator L3
%AC.Actuator.L3.BW = 30;            % [Hz]
AC.Actuator.L3.wn = 97.17;          % [rad/sec]
AC.Actuator.L3.zeta = 0.944;        % [rad/sec]
AC.Actuator.L3.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.L3.PosLim = 25*d2r;   % [rad]
AC.Actuator.L3.NegLim = -25*d2r;  %[rad]

% Flutter suppression L4
%AC.Actuator.L4.BW = 70;            % [Hz]
AC.Actuator.L4.wn = 97.17;          % [rad/sec]
AC.Actuator.L4.zeta = 0.944;        % [rad/sec]
AC.Actuator.L4.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.L4.PosLim = 25*d2r;   % [rad]
AC.Actuator.L4.NegLim = -25*d2r;  %[rad]

% Flutter suppression R1
%AC.Actuator.R1.BW = 70;            % [Hz]
AC.Actuator.R1.wn = 97.17;          % [rad/sec]
AC.Actuator.R1.zeta = 0.944;        % [rad/sec]
AC.Actuator.R1.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R1.PosLim = 25*d2r;   % [rad]
AC.Actuator.R1.NegLim = -25*d2r;  %[rad]

% Right Aileron R2
%AC.Actuator.R2.BW = 30;            % [Hz]
AC.Actuator.R2.wn = 97.17;          % [rad/sec]
AC.Actuator.R2.zeta = 0.944;        % [rad/sec]
AC.Actuator.R2.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R2.PosLim = 25*d2r;   % [rad]
AC.Actuator.R2.NegLim = -25*d2r;  %[rad]

% Elevator R3
%AC.Actuator.R3.BW = 30;            % [Hz]
AC.Actuator.R3.wn = 97.17;          % [rad/sec]
AC.Actuator.R3.zeta = 0.944;        % [rad/sec]
AC.Actuator.R3.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R3.PosLim = 25*d2r;   % [rad]
AC.Actuator.R3.NegLim = -25*d2r;  %[rad]

% Flutter suppression R4
%AC.Actuator.R4.BW = 70;            % [Hz]
AC.Actuator.R4.wn = 97.17;          % [rad/sec]
AC.Actuator.R4.zeta = 0.944;        % [rad/sec]
AC.Actuator.R4.RateLim = 1000*d2r; % [rad/s]
AC.Actuator.R4.PosLim = 25*d2r;   % [rad]
AC.Actuator.R4.NegLim = -25*d2r;  %[rad]

%Throttle
AC.Actuator.throttle.PosLim = 1;  % [nd]
AC.Actuator.throttle.NegLim = 0; %[nd]

%% Configure Sensor Noise Parameters
% same as UltraSticks
% XXX accelerometers in the wings are missing
AC.Sensors.NoiseOn = 1;
AC.Sensors.IMU.p_noise = 0.000001; % rad/s
AC.Sensors.IMU.q_noise = 0.000001; % rad/s
AC.Sensors.IMU.r_noise = 0.000001; % rad/s
AC.Sensors.IMU.hx_noise =150;   % nT 
AC.Sensors.IMU.hy_noise =150;   % nT
AC.Sensors.IMU.hz_noise =80000; % nT
AC.Sensors.IMU.ax_noise =0.0008; % m/s^2
AC.Sensors.IMU.ay_noise =0.004;  % m/s^2
AC.Sensors.IMU.az_noise =0.004;  % m/s^2
AC.Sensors.AirData.ias_noise = 0.001; % m/s
AC.Sensors.AirData.h_noise = 0.02; % m
% pressure based alpha/beta has larger noise; 
AC.Sensors.AirData.alpha_noise = 0.0000005; % rad
AC.Sensors.AirData.beta_noise = 0.0000005; % rad
AC.Sensors.AirData.Pd_noise = 0.00000015; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_noise = 0.0000008; % Kpa, AMS 5812

%% Configure Sensor Bias Parameters
AC.Sensors.IMU.p_bias = 0; % rad/s
AC.Sensors.IMU.q_bias = 0; % rad/s
AC.Sensors.IMU.r_bias = 0; % rad/s
AC.Sensors.IMU.hx_bias = 0;   % nT 
AC.Sensors.IMU.hy_bias = 0;   % nT
AC.Sensors.IMU.hz_bias = 0; % nT
AC.Sensors.IMU.ax_bias = 0; % m/s^2
AC.Sensors.IMU.ay_bias = 0;  % m/s^2
AC.Sensors.IMU.az_bias = 0;  % m/s^2
AC.Sensors.AirData.ias_bias = 0; % m/s
AC.Sensors.AirData.h_bias = 0; % m
AC.Sensors.AirData.alpha_bias = 0; % rad
AC.Sensors.AirData.beta_bias = 0; % rad
AC.Sensors.AirData.Pd_bias = 0; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_bias = 0; % Kpa, AMS 5812

%% Configure Sensor Scale Factor Parameters
AC.Sensors.IMU.p_scf = 1; % rad/s
AC.Sensors.IMU.q_scf = 1; % rad/s
AC.Sensors.IMU.r_scf = 1; % rad/s
AC.Sensors.IMU.hx_scf = 1;   % nT 
AC.Sensors.IMU.hy_scf = 1;   % nT
AC.Sensors.IMU.hz_scf = 1; % nT
AC.Sensors.IMU.ax_scf = 1; % m/s^2
AC.Sensors.IMU.ay_scf = 1;  % m/s^2
AC.Sensors.IMU.az_scf = 1;  % m/s^2
AC.Sensors.AirData.ias_scf = 1; % m/s
AC.Sensors.AirData.h_scf = 1; % m
AC.Sensors.AirData.alpha_scf = 1; % rad
AC.Sensors.AirData.beta_scf = 1; % rad
AC.Sensors.AirData.Pd_scf = 1; % Kpa, AMS 5812
AC.Sensors.AirData.Ps_scf = 1; % Kpa, AMS 5812