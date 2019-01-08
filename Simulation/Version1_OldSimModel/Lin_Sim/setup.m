%% setup.m
% UAV Linear Simulation setup
%
% This script will setup the linear simulation (UAV_Lin.mdl) and call
% trim and linearization routines. Select the desired aircraft in the
% nonlinear simulation (../NL_Sim/setup.m).
%
%
% Calls: ../NL_Sim/setup.m
%        trim_UAV.m
%        linearize_UAV.m
%       
% University of Minnesota 
% Aerospace Engineering and Mechanics 
% Copyright 2011 Regents of the University of Minnesota. 
% All rights reserved.
%
addpath ../Controllers

%% Obtain UAV trim condition matrices and linear model. 
% To update trim targets, edit this m-file.
run ../NL_Sim/setup.m 

%% Simulation sample time
SampleTime = 0.02; % sec

%% Integer Time delay in flight software loop
IntegerTimeDelay = 2; % .04sec

%% Set controller variants
% Each variant corresponds to a different Simulink model that will be
% referenced in the "UAV_Lin/Control Software/Control Software" block.
baseline_control_var = Simulink.Variant('controller_mode == 1');

%% Set controller mode
% Use this variable to quickly change what controller is used in the
% simulation.
%
% 1 = baseline controller (Simulink)

controller_mode = 1;

% Load controller parameters or compile flight code
switch controller_mode
    case 1 %Basleine controller in Simulink
        baseline_gains;   % Declare baseline controller gains
        pitch_gains = [kp_PT, ki_PT, kp_PD];
        roll_gains = [kp_RT, ki_RT, kp_RD];
end

%% Open UAV_Lin model
UAV_Lin
