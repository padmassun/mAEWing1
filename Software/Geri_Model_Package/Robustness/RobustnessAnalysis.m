% #######################################################################
%   Robust Control Design for Active Flutter Suppression
% #######################################################################
%   Julian Theis, Peter Seiler
% ########################################################################
%
% ### ANALYSIS PART ###
% last modified 20/07/2018 Julian Theis
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

%P = GeriFDsysPID2_IO(Kphys.InputName,Kphys.OutputName);
% XXX remove low frequency dynamics. Otherwise stability check fails due to
% open integrators ... this needs some discussion in the group!
RemoveStates = getStatesIndex(GeriFDsysPID2_IO.StateName,{'h','u','theta','beta','phi'});
P = ss([]);
for ii = 1:size(GeriFDsysPID2_IO,3)
P(:,:,ii) = modred(GeriFDsysPID2_IO(C.InputName,C.OutputName,ii),RemoveStates,'truncate');
end

w = {0.01, 1000}; %relevant frequency range for plotting
    
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
    fprintf('Model at airspeed %2.0f m/s\n', Vinf(ii))
[ICM_G(:,ii), ICM_P(:,ii), ICM_D(:,ii), IDM_G(:,ii), IDM_P(:,ii), ...
 OCM_G(:,ii), OCM_P(:,ii), OCM_D(:,ii), ODM_G(:,ii), ODM_P(:,ii), ...
 MMI_G(:,ii), MMI_P(:,ii), MMO_G(:,ii), MMO_P(:,ii), MMIO_G(:,ii), MMIO_P(:,ii)] = ...
 DisplayLoopmargin(P(:,:,ii),C);
end

%calculate Robust and Absolute Flutter Speed

[RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P,OCM_P,ICM_G,OCM_G,Vinf);
fprintf('Robust Flutter Speed: %d \nAbsolute Flutter Speed: %d \n',RFS,AFS)

% plot of minimum classical phase margin at input over airspeed
figure; plot(Vinf,ICM_P,'LineWidth',3); title('Minimum Input Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.InputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; area(vis.RFS_P_x, vis.RFS_P_y); hold off;

% plot of minimum classical gain margin at input over airspeed
figure; semilogy(Vinf,abs(db(ICM_G)),'LineWidth',3); title('Minimum Input Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.InputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]);
hold on; area(vis.RFS_G_x, vis.RFS_G_y); hold off;

% plot of minimum classical phase margin at output over airspeed
figure; plot(Vinf,OCM_P,'LineWidth',3); title('Minimum Output Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]); ylim([0 90]);
hold on; area(vis.RFS_P_x, vis.RFS_P_y); hold off;

% plot of minimum classical gain margin at output over airspeed
figure; semilogy(Vinf,abs(db(OCM_G)),'LineWidth',3); title('Minimum Output Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.OutputName(:),'Location','southwest')
xlim([Vinf(1) Vinf(end)]);
hold on; area(vis.RFS_G_x, vis.RFS_G_y); hold off;

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
DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'g'); %plot gang of six Singular Values

DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'ui'); %plot allowable dynamic multiplicative uncertainty in each input
DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'uo'); %plot allowable dynamic multiplicative uncertainty in each output
DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'ua'); %plot allowable dynamic additive uncertainty
DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'si'); %plot allowable dynamic multiplicative uncertainty in each output
[~, Peaks] = DisplayLoopsens(P(:,:,Vinf==RFS),C,w,'so') %plot allowable dynamic multiplicative uncertainty in each input



%% QUALITATIVE (Root Loci)
% * root loci over airspeed and how they are affected by feedback
% * root loci over airspeed for samples of parametrically perturbed model (structural mode frequency/damping)
% * flutter speed predicted by root-loci (also under uncertainty)

varypzmap(P)
xlim([-60,10])
ylim([-5,60])
sgrid

varypzmap(feedback(P,C))
xlim([-60,10])
ylim([-5,60])
sgrid
%% FURTHER ANALYSIS 
% Mu Analysis with real parameters (I can set up an uncertain model, 
% but we should first agree on the parameters. Plus I think, I require some more time)
% * structured uncertainty descriptions
%   ** real parametric uncertainty in structural mode frequencies and damping ratios
%   ** real parametric uncertainty in air data, i.e. derivatives/coefficients

% TIME SIMULATION WITH RATE LIMITS AND SATURATION
% This is pretty easy to set up but we need to look at the avalaible
% inputs/outputs first.

% TIME SIMULATION (linear/quasi-linear) for multiple fixed airspeeds and/or time-varying airspeed
% This should also be pretty straight forward
% * step responses from elevator (control surfaces not in the loop)
% * step disturbance responses at control surfaces in the loop



