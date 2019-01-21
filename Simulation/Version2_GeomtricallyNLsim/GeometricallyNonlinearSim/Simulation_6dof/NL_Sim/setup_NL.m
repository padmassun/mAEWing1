%% setup_NL  Setup the data required for nonlinear simulation

%% Clear workspace
clear
close all
bdclose all
clc

%% Select Aircraft
AircraftID = 'Geri';

%% Select aerodynamic model type
AeroType = 'DLM';

if ~strcmp(AeroType,'VLM') && ~strcmp(AeroType,'DLM')
    error('AeroType should be "VLM" or "DLM"')
end

%% Add library path and model name
addpath('../Libraries')
mdl = 'NL_Simulation';

%% Configure aircraft
addpath('../Configuration/')
[AC,Env] = ConfigureUAV(AircraftID);

% Index of all CS panels
Idx_PanelIndexAll  = AC.CS.CSPanelIndex_all;

%% Initialize the model
Vinf = 23;      % Aircraft velocity for initialization
StateVals = SetInitial(Vinf,AeroType);

%% Load the model
load_system(mdl)

%% Comment out the unsteady aerodynamics blocks if AeroType = VLM
if strcmp(AeroType,'VLM')
    set_param(['NL_Simulation/Force and Moment /Aerodynamic Model/',...
        'Unsteady Aerodynamics'],'commented','on')
elseif strcmp(AeroType,'DLM')
    set_param(['NL_Simulation/Force and Moment /Aerodynamic Model/',...
        'Unsteady Aerodynamics'],'commented','off')
end

%% Trim the model
set_param(mdl,'LoadExternalInput','off')
set_param(mdl,'LoadInitialState','off')

Op_Trim = ObtainTrimPoint(StateVals,AeroType); 

%% Linearize the model
linmodel = LinearizeModel(mdl,Op_Trim);

%% Postprocessing
%Elevator to Pitch rate response
figure
bodemag(linmodel(2,4),{1e-1 1e2},'b')
grid on
title('Elevator to Pitch Rate')

% Aileron to Roll rate response
figure
bodemag(linmodel(1,7),{1e-1 1e2},'b')
grid on
title('Aileron to Roll Rate')