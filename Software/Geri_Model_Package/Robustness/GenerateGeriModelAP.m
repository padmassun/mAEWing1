%DEMO script: generate an aeroelastic Geri state space model
%Brian Danowsky, Systems Technology, Inc., 2018

function [GeriFDsysPID, GeriFDsysPID2_IO] = GenerateGeriModel(Vmps)

addpath('./..'); %path to model PID box functions including the model function
addpath('../GeriACProps'); %path to model PID box functions including the model function
addpath('../PAAW_IDBox'); %path to model PID box functions including the model function

%load the data
load GeriACProps/Geri_Database
load GeriACProps/GerimassProp
load GeriACProps/GeriaeroPropNOM

%load initial guess and final data -----------------------------------------------------------------

%nominal model (before PID update)
load GeriACProps/GericoeffsNOM
coeffdataNOM = coeffdata;
coeffdataNOM.IputIx = [1:9];
coeffdata.IputIx = [1:9]; %never change this, just means use all inputs

%PID-updated coefficients
indat = load('GeriACProps/GericoeffsPIDUpdate.mat');
coeffdataPIDfull = indat.coeffsPIDfull;

ft2m = 0.3048;

%% generate bare airframe model ====================================================================

% Vmps = 35 %velocity in m/s
aeroProp.V = Vmps/ft2m; %Function uses Vmps and does not use aeroProp.V field but it is good to 
                        %keep it consitent.

%build ss system of final model at a defined velocity with the PID-updated coefficents
GeriFDsysPID = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdataPIDfull);

%identical system to above but with direct individual control surface inputs (this is the system 
%that should be used to check input margins)
GeriFDsysPID2 = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdataPIDfull,'DirectInp');

%The 2 state space systems above have all input, output, and state names labeled with units

%% build model that includes selected I/O and with actuator and sensor dynamics ====================

%this function uses the direct input system (GeriFDsysPID2)

load GeriActuators
engine_lag = tf(1); %perfect engines (for now, can be changed later)
actdata.G_surface_actuator = G_surface_actuator*Delay_25ms; %delays are included with actuators
actdata.engine_lag = engine_lag*Delay_25ms; %delay is included with engine model

load Geri_SensorModels
sensdata.G_sens_IMU = G_sens_IMU;
sensdata.G_sens_Accel = G_sens_Accel;

%select I/O to retain
%ikeep = [1:10]; %all inputs retained
% okeep = [1:31]; %all output retained

ikeep = [1,2,3,4,5,6,7,8,9]; % all flaps + throttle
% ikeep = [1,2,7,8]; %only L1, R1, L4, R4
% okeep = [4 8 13:18]; %only accels
okeep = [1 3 5 7 4 8 13:18]; %only accels

GeriFDsysPID2_IO = NdofwActSens(okeep,ikeep,AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,...
    coeffdataPIDfull,actdata,sensdata);

%% get mode migration and flutter point ============================================================

Vlo = 5;
Vhi = 40;
nVs = 50;
Vrng = [Vlo,Vhi];

chkrng = [20 Inf]; %freq range to check flutter point (it helps to ignore low freq ranges due to 
                   %possibly unstable phugoid that is handled by the SAS)
Vguess = 50; %initial guess at flutter point

%compare frinal with IC
outs = mAEWing1_flutter_analysis(Vrng,nVs,Vmps,coeffdataNOM,coeffdataPIDfull,AEC6,ModeShape,FEM,...
    aeroProp,massProp,chkrng,Vguess);

%% 
% clearvars -except GeriFDsysPID GeriFDsysPID2_IO