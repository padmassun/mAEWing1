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

clear all
close all

%% Define Example System / Controller

Vinfs = 20:0.5:45; %JT: need to define flight speed here for consistency with gain-scheduled controller dimensions
ControllerSelection = 'ILAF'; %'HINF' %'MIDAAS' %'ILAF' 

switch ControllerSelection
    case 'HINF' % HINF Controller
    load('HinfController.mat')

    case 'MIDAAS' % MIDAAS Controller 
    load('..\Controllers\MIDAAS\Geri_MIDAAS_FluttSuppr_Rnd2_SET08_wrolloff.mat');
    clear GeriFDsysPID2_IO %get rid of this to avoid conflicts
%     C.OutputName = {'L1', 'R1', 'L4', 'R4'};
%     C.InputName  = {'qcg','pcg','nzCBfwd','nzCBaft', 'nzLwingfwd', 'nzLwingaft', 'nzRwingfwd', 'nzRwingaft'};
    
    case 'ILAF' % ILAF Controller
    load('..\Controllers\ILAF\GeriAFSC_4x8.mat');
    C = -Cont34_4x8;
    C.OutputName = {'L1', 'R1', 'L4', 'R4'};
    C.InputName  = {'qcg','pcg','nzCBfwd','nzCBaft', 'nzLwingfwd', 'nzLwingaft', 'nzRwingfwd', 'nzRwingaft'};
    TAS = Vinfs;
    SF = zeros(1,1,numel(TAS));
    SF(TAS<=34) = 1;
    SF(TAS>34)=(0.5*TAS(TAS>34)-16); %JT: NOTE THAT THIS GOES BEYOND THE ORIGINAL TABLE, could be easily adapted
    C = SF*C; C.OutputName = {'L1', 'R1', 'L4', 'R4'};
    
    otherwise
        disp('please select HINF, MIDAAS, or ILAF')
end

Vinf = Vinfs; %this needs to be done since MIDAAS loads in a Vinf


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


%% Consider IOs as suggested by BPD 2019-01-18:
AFCS = ss(zeros(size(P,2)-2,size(P,1),numel(Vinf))); %-2 to consider aileron and elevator as single surfaces
AFCS.InputName  = P.OutputName;
AFCS.OutputName = {'L1','R1','aileron','elevator','L4','R4','Thrust'};

switch ControllerSelection
    case {'MIDAAS','HINF'}
        InputAllocP = ss(blkdiag(1,1,[1 -1], [1 1], 1, 1, 1)');
        InputAllocP.InputName = {'L1','R1','aileron','elevator','L4','R4','Thrust'};
        
        OutputAllocP = ss(eye(size(P,1)));
        OutputAllocP.OutputName = P.OutputName;
        
        OutputAllocAFCS = ss(eye(size(AFCS,1)));
        OutputAllocAFCS.OutputName = InputAllocP.InputName;
        OutputAllocAFCS.InputName = AFCS.OutputName;
        
        InputAllocAFCS = ss(eye(size(P,1)));
        InputAllocAFCS.InputName = OutputAllocP.OutputName;
        InputAllocAFCS.OutputName = AFCS.InputName;
        
    case 'ILAF'
        ikeep = [1:3,6];
        
        InputAllocP = ss(blkdiag([1 1],[1 -1], [1 1], 1, 1, 1)');
        InputAllocP.InputName = {'sym BF','aileron','elevator','L4','R4','Thrust'};
        InputAllocP = InputAllocP(:,ikeep); %don't need L4/R4 inputs for ILAF
        InputAllocP.OutputName = P.InputName;
        
        OutputAllocP = ss(eye(size(P,1)-4,size(P,1)));
        OutputAllocP.OutputName = P.OutputName(1:8); %don't need wing accels for ILAF
        OutputAllocP = OutputAllocP([1:4,6:8],:); %ILAF doesn't use qcg
        
        OutputAllocAFCS = ss(blkdiag([1 1],1,1,1,1,1));
        OutputAllocAFCS.InputName = AFCS.OutputName;
        OutputAllocAFCS = OutputAllocAFCS(ikeep,:);
        OutputAllocAFCS.OutputName = InputAllocP.InputName;
        
        InputAllocAFCS = ss(eye(size(P,1),size(P,1)-4));
        InputAllocAFCS.OutputName = AFCS.InputName;
        InputAllocAFCS = InputAllocAFCS(:,[1:4,6:8]); %ILAF doesn't use qcg
        InputAllocAFCS.InputName = OutputAllocP.OutputName;
end
OutputAllocP.InputName = P.OutputName;
InputAllocP.OutputName = P.InputName;

AFCS(C.OutputName,C.InputName,:) = C;    % Flutter Suppression

load BaseLineController
load standard_sos
AFCS('Thrust','u',:) = AutoThrottle;                 % Autothrottle
AFCS({'aileron'},'pcg',:) = RollDamper;       % Roll Damper
AFCS({'aileron'},'phi',:) = RollController;   % Bank Angle Control
AFCS({'elevator'},{'theta','h'},:) = ss(PitchController)*[1 AltitudeController]; % Pitch Angle and Altitude Control


%% Compute Robustness Margins
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
 DisplayLoopmargin( minreal(OutputAllocP*P(:,:,ii)*InputAllocP) , OutputAllocAFCS*AFCS(:,:,ii)*InputAllocAFCS); % minreal here could be avoided; currently used to get rid off actuator states in the elevator/aileron inputs 
end

%calculate Robust and Absolute Flutter Speed

%set thresholds
GMthresh = 6;
PMthresh = 45;

[RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P,OCM_P,ICM_G,OCM_G,Vinf,PMthresh,GMthresh);

InputPhase = ICM_P; InputPhase(InputPhase>=90)=90; 
InputGain = abs(db(ICM_G));
InputGain(ICM_G==0) = -1E2;
InputGainMMI = abs(db(MMI_G));

OutputPhase = OCM_P; OutputPhase(OutputPhase>=90)=90;
OutputGain = abs(db(OCM_G));
OutputGain(OCM_G==0) = -1E2;

RFSIPM = findcrossing(Vinf,min(InputPhase),PMthresh); %RFS using this criteria;
if min(InputPhase(:,Vinf<AFS)) > PMthresh;RFSIPM = AFS;end
RFSIGM = findcrossing(Vinf,min(InputGain),GMthresh); %RFS using this criteria;
if min(InputGain(:,Vinf<AFS)) > GMthresh;RFSIGM = AFS;end
RFSMMI = findcrossing(Vinf,InputGainMMI,GMthresh); %RFS using this criteria; 
if InputGainMMI(:,Vinf<AFS) > GMthresh;RFSMMI = AFS;end
RFSOPM = findcrossing(Vinf,min(OutputPhase),PMthresh); %RFS using this criteria;
if min(OutputPhase(:,Vinf<AFS)) > PMthresh;RFSOPM = AFS;end
RFSOGM = findcrossing(Vinf,min(OutputGain),GMthresh); %RFS using this criteria;
if min(OutputGain(:,Vinf<AFS)) > GMthresh;RFSOGM = AFS;end

RFS = min([RFSIPM,RFSIGM,RFSOPM,RFSOGM]); %based on classical loop-at-a-time
% RFS = min([RFSIPM,RFSIGM,RFSOPM,RFSOGM,RFSMMI]); %based on classical loop-at-a-time and MMI GM

fprintf('Robust Flutter Speed: %2.1f m/s\nAbsolute Flutter Speed: %2.1f m/s',RFS,AFS)
fprintf(['\n   Individual RFS:\n   RFS due to PM at Input: %2.1f m/s' ...
    '\n   RFS due to GM at Input: %2.1f m/s' ...
    '\n   RFS due to PM at Output: %2.1f m/s' ...
    '\n   RFS due to GM at Output: %2.1f m/s' ...
    '\n   RFS due to GM at Multi-Input: %2.1f m/s\n'],RFSIPM,RFSIGM,RFSOPM,RFSOGM,RFSMMI);


%% plot Robust Flutter Margin Results

%set line types
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

% plot of minimum classical phase margin at input over airspeed
figure; 
plot(Vinf,InputPhase,'LineWidth',3); title('Minimum Input Phase Margin'); xlabel('airspeed'); ylabel('degrees');
ylimits = get(gca,'Ylim');
ylims = vis.RFS_P_y;
ylims(ylims==0) = ylimits(1);
hold on
RFS_P_x_IPM = vis.RFS_P_x;
RFS_P_x_IPM(2:3) = RFSIPM;
fill(RFS_P_x_IPM, ylims,[1 0.7 0.7],'LineStyle','none');
plot([AFS AFS],ylimits,'k--','LineWidth',3); 
xlim([Vinf(1) Vinf(end)]); %ylim([0 20]);
grid on
legend([InputAllocP.InputName;['RFS = ' num2str(RFSIPM,'%2.1f') ' m/s'];['AFS = ' num2str(AFS,'%2.1f') ' m/s']],'Location','best')

% plot of minimum classical gain margin at input over airspeed
figure; 
semilogy(Vinf,InputGain,'LineWidth',3); title('Minimum Input Gain Margin'); xlabel('airspeed'); ylabel('dB');
ylimits = get(gca,'Ylim');
ylims = vis.RFS_G_y;
ylims(ylims==0) = min(ylimits(1),1);
RFS_G_x_IGM = vis.RFS_G_x;
RFS_G_x_IGM(2:3) = RFSIGM;
hold on
fill(RFS_G_x_IGM, ylims,[1 0.7 0.7],'LineStyle','none');
plot([AFS AFS],ylimits,'k--','LineWidth',3); 
xlim([Vinf(1) Vinf(end)]); %ylim([0 20]);
grid on
legend([InputAllocP.InputName;['RFS = ' num2str(RFSIGM,'%2.1f') ' m/s'];['AFS = ' num2str(AFS,'%2.1f') ' m/s']],'Location','best')

% plot of Multi-Input Disk Gain Margin at input over airspeed
figure; 
semilogy(Vinf,InputGainMMI,'LineWidth',3); title('Multi-Input Disk Margin (Gain)'); xlabel('airspeed'); ylabel('dB');
ylimits = get(gca,'Ylim');
ylims = vis.RFS_G_y;
ylims(ylims==0) = min(ylimits(1),1);
RFS_G_x_MMI = vis.RFS_G_x;
RFS_G_x_MMI(2:3) = RFSMMI;
hold on
fill(RFS_G_x_MMI, ylims,[1 0.7 0.7],'LineStyle','none');
plot([AFS AFS],ylimits,'k--','LineWidth',3); 
xlim([Vinf(1) Vinf(end)]); %ylim([0 20]);
grid on
legend('Multi-Input Disk Gain Margin',['RFS = ' num2str(RFSMMI,'%2.1f') ' m/s'],['AFS = ' num2str(AFS,'%2.1f') ' m/s'],'Location','best')

%set line types
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
figure; 
plot(Vinf,OutputPhase,'LineWidth',3); title('Minimum Output Phase Margin'); xlabel('airspeed'); ylabel('degrees');
ylimits = get(gca,'Ylim');
ylims = vis.RFS_P_y;
ylims(ylims==0) = ylimits(1);
hold on
RFS_P_x_OPM = vis.RFS_P_x;
RFS_P_x_OPM(2:3) = RFSOPM;
fill(RFS_P_x_OPM, ylims,[1 0.7 0.7],'LineStyle','none');
plot([AFS AFS],ylimits,'k--','LineWidth',3); 
xlim([Vinf(1) Vinf(end)]); %ylim([0 20]);
grid on
legend([OutputAllocP.outputname;['RFS = ' num2str(RFSOPM,'%2.1f') ' m/s'];['AFS = ' num2str(AFS,'%2.1f') ' m/s']],'Location','best')

% plot of minimum classical gain margin at output over airspeed
figure; 
semilogy(Vinf,OutputGain,'LineWidth',3); title('Minimum Output Gain Margin'); xlabel('airspeed'); ylabel('dB');
ylimits = get(gca,'Ylim');
ylims = vis.RFS_G_y;
ylims(ylims==0) = min(ylimits(1),1);
RFS_G_x_OGM = vis.RFS_G_x;
RFS_G_x_OGM(2:3) = RFSOGM;
hold on
fill(RFS_G_x_OGM, ylims,[1 0.7 0.7],'LineStyle','none');
plot([AFS AFS],ylimits,'k--','LineWidth',3); 
xlim([Vinf(1) Vinf(end)]); %ylim([0 20]);
grid on
legend([OutputAllocP.outputname;['RFS = ' num2str(RFSOGM,'%2.1f') ' m/s'];['AFS = ' num2str(AFS,'%2.1f') ' m/s']],'Location','best')

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
DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'g'); %plot gang of six Singular Values

DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'ui'); %plot allowable dynamic multiplicative uncertainty in each input
DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'uo'); %plot allowable dynamic multiplicative uncertainty in each output
DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'ua'); %plot allowable dynamic additive uncertainty
DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'si'); %plot allowable dynamic multiplicative uncertainty in each output
[~, Peaks] = DisplayLoopsens(minreal(P(:,:,Vinf==RFS)*InputAllocP),AFCS(:,:,Vinf==RFS),w,'so') %plot allowable dynamic multiplicative uncertainty in each input



%% QUALITATIVE (Root Loci)
% * root loci over airspeed and how they are affected by feedback
% * root loci over airspeed for samples of parametrically perturbed model (structural mode frequency/damping)
% * flutter speed predicted by root-loci (also under uncertainty)

varypzmap(P)
caxis([Vinf(1) Vinf(end)])
xlim([-60,10])
ylim([0,60])
sgrid

varypzmap(feedback(P*InputAllocP,AFCS))
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


