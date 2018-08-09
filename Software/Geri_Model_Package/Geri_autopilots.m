%This script generates a bare airframe Geri model (both nominal and PID-updated) at a specified 
%airspeed, defines autopilots, and then applies the autopilots to the model with actuator, sensor, 
%and delay dynamics. Plots of loop transfer functions with margins and closed-loop step responses 
%are generated for each autopilot loop at the specified airspeed.
%
%Brian Danowsky, Systems Technology, Inc. 2018

clear all
close all

addpath('PAAW_IDBox'); %path to model PID box functions including the model function

%% load GeriACProps/Geri_Database
load GeriACProps/GerimassProp
load GeriACProps/GeriaeroPropNOM
load GeriACProps/Geri_Database

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

%% generate bare airframe model at specified velocity

Vmps = 27; %velocity in m/s

aeroProp.V = Vmps/ft2m; %Function uses Vmps and does not use aeroProp.V field but it is good to 
                        %keep it consitent.

GeriFDsysNOM = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdataNOM); %nominal 

GeriFDsysPID = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdataPIDfull); %pid-updated


%% apply autopilots

% systest = GeriFDsysNOM; %nominal model
systest = GeriFDsysPID; %PID-updated model

load GeriActuators %load actuators
engine_lag = tf(4,[1 4]); %engines with 250 ms time constant

load Geri_SensorModels.mat %load sensors

%Define Controllers --------------------------------------------------------------------------------

%pitch attitude command -----
Kptht = 0.3;
Kitht = 0.5*Kptht;

deothet = -Kptht*tf([1 Kitht/Kptht],[1 0]);
deothet.inputname = 'theta_e';
deothet.inputunit = 'rad';

%roll attitude command -----
%original
Kpphi = 0.5;
Kiphi = 0.3*Kpphi;
Kdphi = tf(0.01);

%DKS modified
% Kpphi = 0.6;
% Kiphi = 0.05;
% Kdphi = tf(0.01);

daophi = Kpphi*tf([1 Kiphi/Kpphi],[1 0]);
daophi.inputname = 'phi_e';
daophi.inputunit = 'rad';
Kdphi.inputname = 'p_e';
Kdphi.inputunit = 'rad/s';

%auto-throttle -----
Kpu = 12*0.0342;
Kiu = 0.22*Kpu;

dTou = Kpu*tf([1 Kiu/Kpu],[1 0]);
dTou.inputname = 'u_e';
dYou.inputunit = 'ft/s';

%altitude hold -----
Kph = 12*2.59E-4;
thetcoh = tf(Kph);
thetcoh.inputname = 'h_e';
thetcoh.inputunit = 'ft';

%Apply Controllers and analyze ---------------------------------------------------------------------

%autothrottle -----
uodT = systest('u','Thrust');
LTF = G_sens_IMU*uodT*engine_lag*Delay_25ms*dTou;
LTF.outputname = uodT.outputname;
LTF.outputunit = uodT.outputunit;

figure('position',[500 250 560 800]);
subplot(2,1,1)
margin(LTF)
set(gca,'Xlim',[1E-2 1E2])
subplot(2,1,2)
step(feedback(LTF,1));
grid on
garyfyFigure;

%pitch attitude -----
thetode = systest('theta','Sym Mid');
LTF = G_sens_IMU*thetode*G_surface_actuator*Delay_25ms*deothet;
LTF.outputname = thetode.outputname;
LTF.outputunit = thetode.outputunit;

figure('position',[500 250 560 800]);
subplot(2,1,1)
margin(LTF)
set(gca,'Xlim',[1E-2 1E2])
subplot(2,1,2)
step(feedback(LTF,1));
grid on
garyfyFigure;

%altitude -----
thethode = systest({'theta','h'},'Sym Mid');
%h with attitude loop closed
hothetc = feedback(blkdiag(G_sens_IMU,G_sens_IMU)*thethode*deothet,[1 0]); 
hothetc.outputname = thethode.outputname;
hothetc.outputunit = thethode.outputunit;
LTF = hothetc(2,1)*thetcoh;

figure('position',[500 250 560 800]);
subplot(2,1,1)
margin(LTF)
set(gca,'Xlim',[1E-2 1E2])
subplot(2,1,2)
step(feedback(LTF,1));
grid on
garyfyFigure;

%roll attitude -----
phipoda = systest({'phi','p'},'AS IB');
LTF = blkdiag(G_sens_IMU,G_sens_IMU)*phipoda*G_surface_actuator*Delay_25ms*[daophi Kdphi];
LTF.outputname = phipoda.outputname;
LTF.outputunit = phipoda.outputunit;
LTF1 = feedback(LTF,diag([0 1])); LTF1 = LTF1(1,1); %LTF with p loop closed
LTF2 = feedback(LTF,diag([1 0])); LTF2 = LTF2(2,2); %LTF with phi loop closed

figure('position',[500 250 560 800]);
subplot(2,1,1)
margin(LTF1)
set(gca,'Xlim',[1E-2 1E2])
subplot(2,1,2)
step(feedback(LTF1,1));
grid on
garyfyFigure;

figure('position',[500 250 560 470]);
margin(LTF2)
set(gca,'Xlim',[1E-2 1E2])
garyfyFigure;