%% ROBUSTNESS EVALUATION FOR CONTROLLER REDESIGN
%
% based on: 
% #######################################################################
%   Robust Control Design for Active Flutter Suppression
% #######################################################################
%   Julian Theis, Harald Pfifer, Peter Seiler
%   Department of Aerospace Engineering and Mechanics
%   University of Minnesota, Minneapolis 
%   AIAA 2016 SciTech Paper
% ########################################################################
% Control Design for Active Flutter Suppression Controller using Hinf mixed
% sensitivity loopshaping.
%
% ### ANALYSIS PART ###
% last modified 06/06/2018 Julian Theis


%% Define Example System / Controller
load('example_skoll'); %P = Skoll Modell for Demonstration Purposes
C = hinfsyn(augw(P(:,:,4),blkdiag(makeweight(1e5,1,0.5),1,1,1),5*eye(2)*makeweight(0.5,10,1e3))); %this is merely an example controller

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


for ii=1:6
    fprintf('Model at airspeed %2.0f m/s\n', Vinf(ii))
[ICM_G(:,ii), ICM_P(:,ii), ICM_D(:,ii), IDM_G(:,ii), IDM_P(:,ii), ...
 OCM_G(:,ii), OCM_P(:,ii), OCM_D(:,ii), ODM_G(:,ii), ODM_P(:,ii), ...
 MMI_G(:,ii), MMI_P(:,ii), MMO_G(:,ii), MMO_P(:,ii), MMIO_G(:,ii), MMIO_P(:,ii)] = ...
 DisplayLoopmargin(P(:,:,ii),C);
end

%example plot of minimum classical phase margin at input over airspeed
figure; plot(Vinf,OCM_P); title('Minimum Output Phase Margin'); xlabel('airspeed'); ylabel('degrees');legend(P.OutputName(:))
%example plot of symmetric disk margin at input over airspeed
figure; plot(Vinf,mag2db(IDM_G)); title('Input Disk Gain Margin'); xlabel('airspeed'); ylabel('dB');legend(P.InputName(:))

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

DisplayLoopsens(P(:,:,4),C,w,'g'); %plot gang of six Singular Values

DisplayLoopsens(P(:,:,4),C,w,'ui'); %plot allowable dynamic multiplicative uncertainty in each input
DisplayLoopsens(P(:,:,4),C,w,'uo'); %plot allowable dynamic multiplicative uncertainty in each output
DisplayLoopsens(P(:,:,4),C,w,'ua'); %plot allowable dynamic additive uncertainty
DisplayLoopsens(P(:,:,4),C,w,'si'); %plot allowable dynamic multiplicative uncertainty in each output
[~, Peaks] = DisplayLoopsens(P(:,:,4),C,w,'so') %plot allowable dynamic multiplicative uncertainty in each input


%% QUALITATIVE (Root Loci)
% * root loci over airspeed and how they are affected by feedback
% * root loci over airspeed for samples of parametrically perturbed model (structural mode frequency/damping)
% * flutter speed predicted by root-loci (also under uncertainty)

varypzmap(P)
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



