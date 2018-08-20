% #######################################################################
%   Robust Control Design for Active Flutter Suppression
% #######################################################################
%   Julian Theis, Peter Seiler
% ########################################################################
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
 load example_geri %loads the required files to run this script as an example

%%


Vinf = 25:1:45;

% generate complete model of the aircraft at various airspeeds
% XXX still removing altitude state, as it causes problems in the analysis
clear P
for ii=1:numel(Vinf)
    [~, temp] = GenerateGeriModelAP(Vinf(ii)); 
    P(:,:,ii) = temp;
%     P(:,:,ii) = modred(temp([1:2,4:12],:),17,'truncate');
end
GeriFDsysPID2_IO = P; %to be used in AnalysisSimAP
close all; clc

w = {0.01, 1000}; %relevant frequency range for plotting

AFCS = ss(zeros(size(P,2),size(P,1)));
AFCS.InputName = P.OutputName;
AFCS.OutputName = P.InputName;

AFCS(C.OutputName,C.InputName) = C;    % Flutter Suppression

load BaseLineController
AFCS('Thrust','u') = AutoThrottle;                 % Autothrottle
AFCS({'L2','R2'},'p') = [1;-1]*RollDamper;         % Roll Damper
AFCS({'L2','R2'},'phi') = [1;-1]*RollController;   % Bank Angle Control
AFCS({'L3','R3'},{'theta','h'}) = [1;1]*ss(PitchController)*[1 AltitudeController]; % Pitch Angle and Altitude Control

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
    fprintf('\nModel at airspeed %2.0f m/s:\n', Vinf(ii))
[ICM_G(:,ii), ICM_P(:,ii), ICM_D(:,ii), IDM_G(:,ii), IDM_P(:,ii), ...
 OCM_G(:,ii), OCM_P(:,ii), OCM_D(:,ii), ODM_G(:,ii), ODM_P(:,ii), ...
 MMI_G(:,ii), MMI_P(:,ii), MMO_G(:,ii), MMO_P(:,ii), MMIO_G(:,ii), MMIO_P(:,ii)] = ...
 DisplayLoopmargin(P(:,:,ii),AFCS);
end

%calculate Robust and Absolute Flutter Speed

[RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P,OCM_P,ICM_G,OCM_G,Vinf);
fprintf('Robust Flutter Speed: %d \nAbsolute Flutter Speed: %d \n',RFS,AFS)

% plot of minimum classical phase margin at input over airspeed
InputPhase = ICM_P; InputPhase(InputPhase>=90)=90; InputPhase(InputPhase==0)=-Inf;
figure; plot(Vinf,InputPhase,'LineWidth',3); title('Minimum Input Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.InputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; plot([AFS AFS],[0 90],'k--','LineWidth',3); 
area(vis.RFS_P_x, vis.RFS_P_y); hold off;

% plot of minimum classical gain margin at input over airspeed
InputGain = abs(db(ICM_G));
figure; semilogy(Vinf,InputGain,'LineWidth',3); title('Minimum Input Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.InputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 40]);
hold on; plot([AFS AFS],[1 100],'k--','LineWidth',3); 
area(vis.RFS_G_x, vis.RFS_G_y); hold off;

% plot of minimum classical phase margin at output over airspeed
OutputPhase = OCM_P; OutputPhase(OutputPhase>=90)=90; OutputPhase(OutputPhase==0)=-Inf;
figure; plot(Vinf,OutputPhase,'LineWidth',3); title('Minimum Output Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; plot([AFS AFS],[0 90],'k--','LineWidth',3); 
area(vis.RFS_P_x, vis.RFS_P_y); hold off;

% plot of minimum classical gain margin at output over airspeed
OutputGain = abs(db(OCM_G));
figure; semilogy(Vinf,OutputGain,'LineWidth',3); title('Minimum Output Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]);
hold on; plot([AFS AFS],[1 100],'k--','LineWidth',3); 
area(vis.RFS_G_x, vis.RFS_G_y); hold off;

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
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'g'); %plot gang of six Singular Values

DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'ui'); %plot allowable dynamic multiplicative uncertainty in each input
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'uo'); %plot allowable dynamic multiplicative uncertainty in each output
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'ua'); %plot allowable dynamic additive uncertainty
DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'si'); %plot allowable dynamic multiplicative uncertainty in each output
[~, Peaks] = DisplayLoopsens(P(:,:,Vinf==RFS),AFCS,w,'so') %plot allowable dynamic multiplicative uncertainty in each input



%% QUALITATIVE (Root Loci)
% * root loci over airspeed and how they are affected by feedback
% * root loci over airspeed for samples of parametrically perturbed model (structural mode frequency/damping)
% * flutter speed predicted by root-loci (also under uncertainty)

varypzmap(P)
xlim([-60,10])
ylim([-5,60])
sgrid

varypzmap(feedback(P,AFCS))
xlim([-60,10])
ylim([-5,60])
sgrid
%% FURTHER ANALYSIS 
% perform time-domain simulation in Simulink using rate-limited and
% saturated control surfaces and the autopilot modules in the loop
open AnalysisSimAp



