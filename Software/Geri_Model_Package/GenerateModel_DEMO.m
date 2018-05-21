%DEMO script: generate an aeroelastic Geri state space model
%Brian Danowsky, Systems Technology, Inc., 2018

clear all
close all

addpath('PAAW_IDBox'); %path to model PID box functions including the model function

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

%generate a model ==================================================================================

ft2m = 0.3048;

Vmps = 23; %velocity in m/s
aeroProp.V = Vmps/ft2m; %Function uses Vmps and does not use aeroProp.V field but it is good to keep it consitent.

%build ss system of final model with the PID-updated coefficents
GeriFDsysPID = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdataPIDfull);

%get mode migration and flutter point ==============================================================

Vlo = 5;
Vhi = 40;
nVs = 50;
Vrng = [Vlo,Vhi];

chkrng = [20 Inf]; %freq range to check flutter point (it helps to ignore low freq ranges due to possibly unstable phugoid that is handled by the SAS)
Vguess = 50; %initial guess at flutter point

%compare frinal with IC
outs = mAEWing1_flutter_analysis(Vrng,nVs,Vmps,coeffdataNOM,coeffdataPIDfull,AEC6,ModeShape,FEM,aeroProp,massProp,chkrng,Vguess);
