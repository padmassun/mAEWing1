% #######################################################################
%   Robust Control Design for Active Flutter Suppression
% #######################################################################
%   Julian Theis, Peter Seiler
% #######################################################################
%
% ### ANALYSIS PART ###
% last modified 2018-08-15 Julian Theis
%
% RUN PART1_DESIGN_ROBUSTFLUTTERSUPPRESSION FIRST
% uses variables in workspace, requires 
% GeriFDsysPID2_IO : model with physical inputs/outputs
% C : controller with physical inputs/outputs
% Vinf : vector of airspeed
%
% The controller needs to have the correct naming of inputs and outputs
% (matching the GeriFDsysPID2_IO model)
%% Define Example System / Controller

Vinf = 20:0.5:45; %JT: need to define flight speed here for consistency with gain-scheduled controller dimensions
ControllerSelection = 'MIDAAS' %'HINF' %'ILAF' 

switch ControllerSelection
    case 'HINF' % HINF Controller
    load('HinfController.mat')

    case 'MIDAAS' % MIDAAS Controller 
    load('..\Controllers\MIDAAS\Geri_MIDAAS_FluttSuppr_Rnd2_SET08_wrolloff.mat');
    clear GeriFDsysPID2_IO %get rid of this to avoid conflicts
    C.OutputName = {'L1', 'R1', 'L4', 'R4'};
    C.InputName  = {'qcg','pcg','nzCBfwd','nzCBaft', 'nzLwingfwd', 'nzLwingaft', 'nzRwingfwd', 'nzRwingaft'};
    
    case 'ILAF' % ILAF Controller
    load('..\Controllers\ILAF\GeriAFSC_4x8.mat');
    C = -Cont34_4x8;
    C.OutputName = {'L1', 'R1', 'L4', 'R4'};
    C.InputName  = {'qcg','pcg','nzCBfwd','nzCBaft', 'nzLwingfwd', 'nzLwingaft', 'nzRwingfwd', 'nzRwingaft'};
    TAS = Vinf;
    SF = zeros(1,1,numel(TAS));
    SF(TAS<=34) = 1;
    SF(TAS>34)=(0.5*TAS(TAS>34)-16); %JT: NOTE THAT THIS GOES BEYOND THE ORIGINAL TABLE, could be easily adapted
    C = SF*C; C.OutputName = {'L1', 'R1', 'L4', 'R4'};
    
    otherwise
        disp('please select HINF, MIDAAS, or ILAF')
end

%% load some preliminary model stuff
if size(C,3) == 1 %if controller not gain scheduled
    C(:,:,1:numel(Vinf)) = C; % expand controller into scheduling dimension for compatability
end

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

load GeriActuators
engine_lag = tf(1); %tf(4,[1 4]); %tf(1); %perfect engines (for now, can be changed later)
actdata.G_surface_actuator = G_surface_actuator*Delay_25ms; %delays are included with actuators
actdata.engine_lag = engine_lag*Delay_25ms; %delay is included with engine model

load Geri_SensorModels
sensdata.G_sens_IMU = G_sens_IMU;
sensdata.G_sens_Accel = G_sens_Accel;

ikeep = [1,2,3,4,5,6,7,8,9]; % all flaps + throttle
% okeep = [1 3 5 7 4 8 13:18]; %uses q and p (mean axis rates)
okeep = [1 3 5 7 10 11 13:18]; %uses qcg and pcg (sensor rates, more correct)

%bare airframe as a function of velocity 
gerifunc = @(Vmps)NdofwActSens(okeep,ikeep,AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,...
    coeffdataPIDfull,actdata,sensdata);


%% generate models

% generate complete model of the aircraft at various airspeeds
clear P
for ii=1:numel(Vinf)
%     [~, temp] = GenerateGeriModelAP(Vinf(ii));
    temp = gerifunc(Vinf(ii));
    P(:,:,ii) = temp;
%     P(:,:,ii) = modred(temp([1:2,4:12],:),17,'truncate');
end
GeriFDsysPID2_IO = P; %to be used in AnalysisSimAP
close all; clc

w = {0.01, 1000}; %relevant frequency range for plotting

AFCS = ss(zeros(size(P,2),size(P,1),numel(Vinf)));
AFCS.InputName  = P.OutputName;
AFCS.OutputName = P.InputName;

AFCS(C.OutputName,C.InputName,:) = C;    % Flutter Suppression

load BaseLineController
load standard_sos
AFCS('Thrust','u',:) = AutoThrottle;                 % Autothrottle
AFCS({'L2','R2'},'pcg',:) = [1;-1]*RollDamper;       % Roll Damper
AFCS({'L2','R2'},'phi',:) = [1;-1]*RollController;   % Bank Angle Control
AFCS({'L3','R3'},{'theta','h'},:) = [1;1]*ss(PitchController)*[1 AltitudeController]; % Pitch Angle and Altitude Control


%% Robustness Margins
% Robustness margins are calculated using the LOOPMARGIN command (called
% inside the function DisplayLoopmargin which also prints the data to the workspace)
%
% The considered metrics here are 
% - the classical single loop margins at both input and output 
% - the multi loop input disk margin 
%   (simultaneous gain and phase perturbation in all inputs)
% - the multi loop output disk margin 
%   (simultaneous gain and phase perturbation in all outputs)
% - the multi loop output disk margin 
%   (simultaneous gain and phase perturbation in all outputs)
% - the multi loop IO disk margin 
%   (simultaneous gain and phase perturbation in all inputs AND outputs)
%
% Covers the following points from our discussion:
%  * single loop margins for all individual feedback loops that are closed by the respective controllers
%  ** classical gain and phase margins
%  ** (symmetric) disk margins
%  ** delay margins
%  * multi loop margins for all simultaneously closed feedback loops by the respective controller
%  ** (symmetric) disk margins



for ii=1:numel(Vinf)
    fprintf('\nModel at airspeed %2.1f m/s:\n', Vinf(ii))
[ICM_G(:,ii), ICM_P(:,ii), ICM_D(:,ii), IDM_G(:,ii), IDM_P(:,ii), ...
 OCM_G(:,ii), OCM_P(:,ii), OCM_D(:,ii), ODM_G(:,ii), ODM_P(:,ii), ...
 MMI_G(:,ii), MMI_P(:,ii), MMO_G(:,ii), MMO_P(:,ii), MMIO_G(:,ii), MMIO_P(:,ii)] = ...
 DisplayLoopmargin(P(:,:,ii),AFCS(:,:,ii));
end


% Roll Control AP Loop Robustness Check %JT: This is a little ad hoc, but
% should work
contrf = 1:9; measf = 1:12; contr = [3 4]; meas = [4 6]; alloc = [1 -1];
Paug = feedback(P,minreal(AFCS(contrf(not(ismember(contrf,contr))),measf(not(ismember(measf,meas))))),contrf(not(ismember(contrf,contr))),measf(not(ismember(measf,meas))));
Paug = minreal(Paug(meas,contr)*alloc');
Paug.InputName = {'L2-R2'};
AP = alloc*minreal(AFCS(contr,meas));
AP.OutputName = {'L2-R2'};
for ii=1:numel(Vinf)
    fprintf('\nRoll AP: Model at airspeed %2.1f m/s:\n', Vinf(ii))
    [ICM_G_roll(:,ii), ICM_P_roll(:,ii), ICM_D_roll(:,ii), IDM_G_roll(:,ii), IDM_P_roll(:,ii), ...
     OCM_G_roll(:,ii), OCM_P_roll(:,ii), OCM_D_roll(:,ii), ODM_G_roll(:,ii), ODM_P_roll(:,ii), ...
     MMI_G_roll(:,ii), MMI_P_roll(:,ii), MMO_G_roll(:,ii), MMO_P_roll(:,ii), MMIO_G_roll(:,ii), MMIO_P_roll(:,ii)] = ...
    DisplayLoopmargin(Paug(:,:,ii),AP(:,:,ii));
end


% Pitch Control AP Loop Robustness Check 
contrf = 1:9; measf = 1:12; contr = [5 6]; meas = [2 3]; alloc = [1 1];
Paug = feedback(P,minreal(AFCS(contrf(not(ismember(contrf,contr))),measf(not(ismember(measf,meas))))),contrf(not(ismember(contrf,contr))),measf(not(ismember(measf,meas))));
Paug = minreal(Paug(meas,contr)*alloc');
Paug.InputName = {'L3+R3'};
AP = alloc*minreal(AFCS(contr,meas));
AP.OutputName = {'L3+R3'};
for ii=1:numel(Vinf)
    fprintf('\nPitch AP: Model at airspeed %2.1f m/s:\n', Vinf(ii))
    [ICM_G_pitch(:,ii), ICM_P_pitch(:,ii), ICM_D_pitch(:,ii), IDM_G_pitch(:,ii), IDM_P_pitch(:,ii), ...
     OCM_G_pitch(:,ii), OCM_P_pitch(:,ii), OCM_D_pitch(:,ii), ODM_G_pitch(:,ii), ODM_P_pitch(:,ii), ...
     MMI_G_pitch(:,ii), MMI_P_pitch(:,ii), MMO_G_pitch(:,ii), MMO_P_pitch(:,ii), MMIO_G_pitch(:,ii), MMIO_P_roll(:,ii)] = ...
    DisplayLoopmargin(Paug(:,:,ii),AP(:,:,ii));
end

ICM_P = [ICM_P; ICM_P_roll; ICM_P_pitch];
ICM_G = [ICM_G; ICM_G_roll; ICM_G_pitch];

%calculate Robust and Absolute Flutter Speed
[RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P,OCM_P,ICM_G,OCM_G,Vinf);
fprintf('Robust Flutter Speed: %2.1f \nAbsolute Flutter Speed: %2.1f \n',RFS,AFS)


% plot of minimum classical phase margin at input over airspeed
set(groot,'DefaultAxesColorOrder',[     0.5151    0.0482    0.6697
                                        0.4937    0.2780    0.9119
                                        0.3999    0.4564    0.9832
                                        0.3001    0.6139    0.8594
                                        0.2301    0.7377    0.6762
                                        0.2968    0.8270    0.4643
                                        0.3778    0.8968    0.2928
                                        0.6180    0.9255    0.3314
                                        0.8000    0.9255    0.3529],...
      'DefaultAxesLineStyleOrder','-|-.|:|--')
  
InputPhase = ICM_P; InputPhase(InputPhase>=90)=90; InputPhase(InputPhase==0)=-Inf;
figure; plot(Vinf,InputPhase,'LineWidth',3); title('Minimum Input Phase Margin'); xlabel('airspeed'); ylabel('degrees');
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; plot([AFS AFS],[0 90],'k--','LineWidth',3); 
fill(vis.RFS_P_x, vis.RFS_P_y,[1 0.7 0.7],'LineStyle','none'); hold off;
legend([P.InputName(:);'Roll AP';'Pitch AP';['AFS = ' num2str(AFS,'%2.1f') ' m/s'];['RFS = ' num2str(RFS,'%2.1f') ' m/s']],'Location','best')
grid on

% plot of minimum classical gain margin at input over airspeed
InputGain = abs(db(ICM_G));
figure; semilogy(Vinf,InputGain,'LineWidth',3); title('Minimum Input Gain Margin'); xlabel('airspeed'); ylabel('dB');
xlim([Vinf(1) Vinf(end)]); ylim([0 40]);
hold on; plot([AFS AFS],[1 100],'k--','LineWidth',3); 
ylims = vis.RFS_G_y;
ylims(ylims==0) = 1;
fill(vis.RFS_G_x, ylims,[1 0.7 0.7],'LineStyle','none'); hold off;
ylim([1 100]);
legend([P.InputName(:);'Roll AP';'Pitch AP';['AFS = ' num2str(AFS,'%2.1f') ' m/s'];['RFS = ' num2str(RFS,'%2.1f') ' m/s']],'Location','best')
grid on


set(groot,'DefaultAxesColorOrder',[     0.5151    0.0482    0.6697
                                        0.5139    0.2199    0.8542
                                        0.4451    0.3603    0.9842
                                        0.3849    0.4873    0.9746
                                        0.3092    0.6009    0.8722
                                        0.2246    0.7011    0.7448
                                        0.2527    0.7706    0.6044
                                        0.3030    0.8330    0.4453
                                        0.3553    0.8867    0.2935
                                        0.4938    0.9181    0.3081
                                        0.6713    0.9255    0.3375
                                        0.8000    0.9255    0.3529])
                                    
% plot of minimum classical phase margin at output over airspeed
OutputPhase = OCM_P; OutputPhase(OutputPhase>=90)=90; OutputPhase(OutputPhase==0)=-Inf;
figure; plot(Vinf,OutputPhase,'LineWidth',3); title('Minimum Output Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; plot([AFS AFS],[0 90],'k--','LineWidth',3); 
% area(vis.RFS_P_x, vis.RFS_P_y); hold off;
fill(vis.RFS_P_x, vis.RFS_P_y,[1 0.7 0.7],'LineStyle','none'); hold off;
legend([P.OutputName(:);'Roll AP';'Pitch AP';['AFS = ' num2str(AFS,'%2.1f') ' m/s'];['RFS = ' num2str(RFS,'%2.1f') ' m/s']],'Location','best')
grid on

% plot of minimum classical gain margin at output over airspeed
OutputGain = abs(db(OCM_G));
figure; semilogy(Vinf,OutputGain,'LineWidth',3); title('Minimum Output Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]);
hold on; plot([AFS AFS],[1 100],'k--','LineWidth',3); 
% area(vis.RFS_G_x, vis.RFS_G_y); hold off;
ylims = vis.RFS_G_y;
ylims(ylims==0) = 1;
fill(vis.RFS_G_x, ylims,[1 0.7 0.7],'LineStyle','none');
ylim([1 100])
legend([P.OutputName(:);['AFS = ' num2str(AFS,'%2.1f') ' m/s'];['RFS = ' num2str(RFS,'%2.1f') ' m/s']],'Location','best')
grid on

set(groot,'defaultAxesColorOrder','remove')
%% Closed-Loop Transfer Function Analysis
% All relevant closed-loop (or broken loop) transfer functions are 
% calculated by LOOPSENS:
% So  : output sensitivity (I+P*C)^-1     with plant P and compensator C
% Si  : input sensitivity (I+C*P)^-1
% To  : complementary output sensitivity P*C(I+P*C)^-1
% Ti  : complementary input sensitivity C*P(I+C*P)^-1
% CSo : controller sensitivity 
% PSi : sensitivity to input disturbances

% Covers the following points from our discussion:
% * multi loop margins for all simultaneously closed feedback loops by the respective controller
%   ** (non-symmetric) disk margins, i.e. peaks of S and T
% * structured uncertainty descriptions
% ** combinations of dynamic actuator, plant, and sensor uncertainty

% check at robust flutter speed
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'g'); %plot gang of six Singular Values

DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'ui'); %plot allowable dynamic multiplicative uncertainty in each input
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'uo'); %plot allowable dynamic multiplicative uncertainty in each output
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'ua'); %plot allowable dynamic additive uncertainty
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'si'); %plot allowable dynamic multiplicative uncertainty in each output
[~, Peaks] = DisplayLoopsens(P(:,:,Vinf==RFS),AFCS(:,:,Vinf==RFS),w,'so') %plot allowable dynamic multiplicative uncertainty in each input



%% QUALITATIVE (Root Loci)
% * root loci over airspeed and how they are affected by feedback
% * root loci over airspeed for samples of parametrically perturbed model (structural mode frequency/damping)
% * flutter speed predicted by root-loci (also under uncertainty)

varypzmap(P)
caxis([Vinf(1) Vinf(end)])
xlim([-60,10])
ylim([0,60])
sgrid

varypzmap(feedback(P,AFCS))
caxis([Vinf(1) Vinf(end)])
xlim([-60,10])
ylim([0,60])
sgrid


%% FURTHER ANALYSIS 
% perform time-domain simulation in Simulink using rate-limited and
% saturated control surfaces and the autopilot modules in the loop

%define rate limits in deg/s
RLrise = 30;
RLfall = -30;

%define surface saturation limits in deg/s
Sathi = 30;
Satlw = -30;

%build Controller for sim that may have I/O in different order
% Csim = ss(zeros(size(C)));
% Csim.outputname = P.inputname([1 2 7 8]);
% Csim.inputname = P.outputname([5:12]);
% Csim(C.OutputName,C.InputName) = C(:,:,Vinf==RFS); % JT: This must still be changed for Simulink Gain-Scheduled Simulation
% open AnalysisSimAP
% sim('AnalysisSimAP');

%build Controller for sim that may have I/O in different order incl gain
%scheduling
Csim = ss(zeros(size(C)));
Csim.outputname = P.inputname([1 2 7 8]);
Csim.inputname = P.outputname([5:12]);
Csim(C.OutputName,C.InputName,:) = C(:,:,:);
Csim_discrete = c2d(Csim,1/151,'tustin');
open AnalysisSimAP_2016a
sim('AnalysisSimAP_2016a');

%disp('...waiting for simulation to finish...')
%pause(60) %wait for simulation to finish. Is there a smarter way to do this?


figure
subplot(411)
plot(Sim_SurfaceDeflections.time,Sim_SurfaceDeflections.signals(1).values,'LineWidth',3)
grid on
ylabel('Body Flaps [deg]'), xlabel('Time [s]'), legend('left','right')
subplot(412)
plot(Sim_SurfaceDeflections.time,Sim_SurfaceDeflections.signals(2).values,'LineWidth',3)
grid on
ylabel('Inboard Flaps [deg]'), xlabel('Time [s]'), legend('left','right')
subplot(413)
plot(Sim_SurfaceDeflections.time,Sim_SurfaceDeflections.signals(3).values,'LineWidth',3)
grid on
ylabel('Midboard Flaps [deg]'), xlabel('Time [s]'), legend('left','right')
subplot(414)
plot(Sim_SurfaceDeflections.time,Sim_SurfaceDeflections.signals(4).values,'LineWidth',3)
grid on
ylabel('Outboard Flaps [deg]'), xlabel('Time [s]'), legend('left','right')

figure
subplot(211)
plot(Sim_Accels.time,Sim_Accels.signals(1).values,'LineWidth',3)
grid on
ylabel('Fore Accelerometers'), xlabel('Time [s]'), legend('left','center','right')
subplot(212)
plot(Sim_Accels.time,Sim_Accels.signals(2).values,'LineWidth',3)
grid on
ylabel('Aft Accelerometers'), xlabel('Time [s]'), legend('left','center','right')

figure
subplot(411)
plot(Sim_ThetaPhi.time,Sim_ThetaPhi.signals(1).values,'LineWidth',3)
grid on
ylabel('Theta [deg]'), xlabel('Time [s]')
subplot(412)
plot(Sim_ThetaPhi.time,Sim_ThetaPhi.signals(2).values,'LineWidth',3)
grid on
ylabel('Phi [deg]'), xlabel('Time [s]')
subplot(413)
plot(Sim_Rates.time,Sim_Rates.signals(1).values,'LineWidth',3)
grid on
ylabel('qcg [deg/s]'), xlabel('Time [s]')
subplot(414)
plot(Sim_Rates.time,Sim_Rates.signals(2).values,'LineWidth',3)
grid on
ylabel('pcg [deg/s]'), xlabel('Time [s]')

figure
subplot(211)
plot(Sim_uh.time,Sim_uh.signals(1).values,'LineWidth',3)
grid on
ylabel('u'), xlabel('Time [s]')
subplot(212)
plot(Sim_uh.time,Sim_uh.signals(2).values,'LineWidth',3)
grid on
ylabel('h'), xlabel('Time [s]')

figure
subplot(211)
plot(Sim_Rollcomm.time,Sim_Rollcomm.signals.values,'LineWidth',3)
grid on
ylabel('Roll Command [deg]'), xlabel('Time [s]')
subplot(212)
plot(Sim_Pitchcomm.time,Sim_Pitchcomm.signals.values,'LineWidth',3)
grid on
ylabel('Pitch Command [deg]'), xlabel('Time [s]')

figure;
for ind = 1:size(P,2)-1
    subplot(size(P,2)-1,1,ind)
    plot(Sim_RLerror.time,Sim_RLerror.signals.values(:,ind),'LineWidth',3)
    ylabel([P.inputname{ind}, ' [' P.inputunit{ind} ']']);
    grid on
    if ind == 1
        title('Rate Limit Error')
    end
end
xlabel('Time [s]')

figure;
for ind = 1:size(P,2)-1
    subplot(size(P,2)-1,1,ind)
    plot(Sim_SatError.time,Sim_SatError.signals.values(:,ind),'LineWidth',3)
    ylabel([P.inputname{ind}, ' [' P.inputunit{ind} ']']);
    grid on
    if ind == 1
        title('Saturation Limit Error')
    end
end
xlabel('Time [s]')


