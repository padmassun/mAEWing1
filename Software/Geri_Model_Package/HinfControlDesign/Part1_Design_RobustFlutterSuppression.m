% #######################################################################
%   Robust Control Design for Active Flutter Suppression
% #######################################################################
%   Julian Theis & Peter Seiler
% ########################################################################
% Control Design for Active Flutter Suppression Controller using Hinf mixed
% sensitivity loopshaping.
%
% ### DESIGN AND SYNTHESIS PART ###
%
% last modified 20/07/2018 Julian Theis


clear all; close all; clc
w = {1,300}; 
if exist('garyfyFigure','file')
    gopts = garyfyFigureOptions; gopts.LineWidth = 1.5;
end
bopt = bodeoptions; bopt.PhaseMatching = 'on'; bopt.PhaseMatchingFreq = 0.1;

%% Aircraft Model                     

% define grid of airspeed for evaluation model
VV = 25:1:45;


% generate complete model of the aircraft at various airspeeds
clear GeriFDsysPID GeriFDsysPID2_IO
for ii=1:numel(VV)
    [GeriFDsysPID(:,:,ii), GeriFDsysPID2_IO(:,:,ii)] = GenerateGeriModel(VV(ii));
end
close all; clc

% select Inputs and Outputs for Control Design 
SelectedInputs   = {'Sym BF','Sym OB', 'AS Ob'};
SelectedOutputs  = {'p', 'q', 'nzCBaft','nzCBfwd','nzRwingaft','nzLwingaft','nzRwingfwd','nzLwingfwd'};


% plot selected model
sys_openloop0 = GeriFDsysPID(SelectedOutputs,SelectedInputs);
figure(1); bodemag(sys_openloop0,w,bopt)

% remove altitude, forward velocity, pitch angle, sideslip angle, bank
% angle and yaw rate from model
RemoveStates = getStatesIndex(sys_openloop0.StateName,{'h','u','theta','beta','phi','r'}); %p needed due to asymmetry
clear sys_openloop1
for ii=1:numel(VV)
sys_openloop1(:,:,ii) = modred(sys_openloop0(:,:,ii),RemoveStates,'truncate');
damp(sys_openloop1(:,:,ii))
end

% remove 5th and 6th elastic mode from model
RemoveStates = getStatesIndex(sys_openloop1.StateName,{'eta5','eta5dt','eta6','eta6dt'});
clear sys_openloop
for ii=1:numel(VV)
sys_openloop(:,:,ii) = modred(sys_openloop1(:,:,ii),RemoveStates,'residualize');
damp(sys_openloop(:,:,ii))
end
figure(1); hold on; bodemag(sys_openloop,w,bopt); hold off;


%%
    InputAlloc = ss([1 0 0 0 0  1  0  0  0 0;
                  1 0 0 0 0 -1  0  0  0 0;
                  0 1 0 0 0  0  1  0  0 0;
                  0 1 0 0 0  0 -1  0  0 0;
                  0 0 1 0 0  0  0  1  0 0;
                  0 0 1 0 0  0  0 -1  0 0;
                  0 0 0 1 0  0  0  0  1 0;
                  0 0 0 1 0  0  0  0 -1 0;
                  0 0 0 0 1  0  0  0  0 0;
                  0 0 0 0 0  0  0  0  0 1]);
    InputAlloc.outputname = {'L1','R1','L2','R2','L3','R3','L4','R4','Thrust','wGust'};
    InputAlloc.inputname = GeriFDsysPID.InputName;           
    InputAllocInv = inv(InputAlloc);
    
    
    SelectedInputs = {'L1','R1','L4','R4'};
    sys_openloop=sys_openloop*InputAllocInv(sys_openloop.InputName,SelectedInputs);
    
%%  Select an output blend to be used by the controller
% OutputBlend = ss(blkdiag(1,1,1,[0.5 0.5 0 0; 0 0 0.5 0.5; 0.5 -0.5 0 0; 0 0 0.5 -0.5])); % 
% OutputBlend.InputName = sys_openloop.OutputName;
% OutputBlend.OutputName = {'q','nzc_aft','nzc_fwd','nzsym_aft','nzsym_fwd','nzasym_aft','nzasym_fwd'};

% DesignModel = OutputBlend*sys_openloop;


OutputBlend = ss(eye(8)); % 
OutputBlend.InputName = sys_openloop.OutputName;
OutputBlend.OutputName = sys_openloop.OutputName;

DesignModel = OutputBlend*sys_openloop;

%% + Actuator and Sensor Dynamics
% add sensor and actuator dynamics if desired
act_sens = true; % false;


figure(2); bodemag(DesignModel,w,bopt)
if act_sens
     load('GeriActuators')
     load('Geri_SensorModels')
     DesignModel = blkdiag(G_sens_IMU*eye(2),G_sens_Accel*eye(6))*DesignModel*blkdiag(eye(4)*G_surface_actuator*Delay_25ms);
     DesignModel.OutputName = OutputBlend.OutputName; DesignModel.InputName = sys_openloop.InputName;
     sys_openloop = blkdiag(G_sens_IMU*eye(2),G_sens_Accel*eye(6))*sys_openloop*blkdiag(eye(4)*G_surface_actuator*Delay_25ms);
     sys_openloop.OutputName = SelectedOutputs; sys_openloop.InputName = SelectedInputs; 
else
    disp('! no actuator and sensor dynamics includes !')
end
figure(2); hold on; bodemag(DesignModel,w,bopt); hold off

% could also do a lower order equivalent model here

%% H-Infinity Controller Design

% select airspeed for design model
designspeed = 40; %40

% Select scaling/tuning matrix "maximum errors"
% pitch rate is in rad/s, all accels are in ft/s^2 -> 1 g
De = 1*blkdiag(1,1,32,32,32,32,32,32);

% Select scaling/tuning matrix "maximum inputs"
% all inputs are in rad
Du = blkdiag(1/180*pi,2/180*pi,2/180*pi,1/180*pi);  %2degs


% Select bandpass filters for control effort (control active between 10 and
% 100 rad/s
Wu = blkdiag(eye(2)*bandweight(10,70),eye(2)*bandweight(5,70));  %10/70 10/30

% Select weights for modal velocities
Weta = 1*blkdiag(1/20,1/20);

% Select disturbance weight
Dd = Du; %Du; %this could be a dedicated Dd different from Du

syn_mod = modalform( DesignModel(:,:,VV==designspeed) );

% add modal velocities of structural modes as performance outputs
syn = add_state_output(syn_mod, [18 20]);
[nmeas, ncont] = size(DesignModel);
nperf = size(syn,1)-nmeas;


% build generalized plant. see description of genplant
Gweighted = genplant(syn,De^-1,Wu/Du,Dd,De,[],Weta);

% show weighted modal sensitivities (scale such that peak slightly above
% 0db. larger to emphasize. 
figure(4); bodemag(Gweighted(nmeas+ncont+1:nmeas+ncont+nperf,1:ncont),{10,300})

%%
% A controller is synthesized using the hinfsyn command. 
% A suboptimal two-step procedure and scaling are applied to improve numerics.
[~,~,gamopt,~] = hinfsyn(ssbal(Gweighted),nmeas,ncont,'Method','ric');
[K,FGK,gam,info] = hinfsyn(ssbal(Gweighted),nmeas,ncont,'Method','ric',...
                           'GMIN',gamopt*1.1,'GMAX',gamopt*1.1);  
gam                       

K = K*OutputBlend;
K.InputName= sys_openloop.OutputName;
K.OutputName = sys_openloop.InputName;
K.StateName(:) = {'K'};

% Plot controller
disp('controller dynamics')
damp(K)
figure(5)
bodemag(K,{0.1,300},bopt);grid;
if exist('garyfyFigure','file');garyfyFigure;end;

% Plot the modal sensitivites open-loop vs closed-loop
figure(6);bodemag(Gweighted(nmeas+ncont+1:nmeas+ncont+nperf,1:ncont),FGK(nmeas+ncont+1:nmeas+ncont+nperf,1:ncont),{10,300})




          
%% Comparing the pole locations of the closed-loop with and without flutter
% suppression 

varypzmap(sys_openloop,'gray')
hold on; pzmap(sys_openloop(:,:,VV==designspeed),'r'); hold off;
xlim([-60,10])
ylim([-5,60])
sgrid


varypzmap(feedback(sys_openloop,K),'gray')
hold on; pzmap(feedback(sys_openloop(:,:,VV==designspeed),K),'r'); hold off;
xlim([-60,10])
ylim([-5,60])
sgrid



%% Define System / Controller

Kphys = K;

RemoveStates = getStatesIndex(GeriFDsysPID2_IO.StateName,{'h','u','theta','phi'});
P = ss([]);
for ii = 1:size(GeriFDsysPID2_IO,3)
P(:,:,ii) = modred(GeriFDsysPID2_IO(Kphys.InputName,Kphys.OutputName,ii),RemoveStates,'truncate');
end

C = Kphys;   % renaming so that following could remains the same
Vinf = VV;

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
    fprintf('\nModel at airspeed %2.0f m/s:\n', Vinf(ii))
[ICM_G(:,ii), ICM_P(:,ii), ICM_D(:,ii), IDM_G(:,ii), IDM_P(:,ii), ...
 OCM_G(:,ii), OCM_P(:,ii), OCM_D(:,ii), ODM_G(:,ii), ODM_P(:,ii), ...
 MMI_G(:,ii), MMI_P(:,ii), MMO_G(:,ii), MMO_P(:,ii), MMIO_G(:,ii), MMIO_P(:,ii)] = ...
 DisplayLoopmargin(P(:,:,ii),C);
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
hold on; plot([AFS AFS],[0 90],'k--','LineWidth',3); 
area(vis.RFS_G_x, vis.RFS_G_y); hold off;














%% IGNORE FOR NOW
%
% % The step response to a pitch command further demonstrates the
% % effectiveness of the proposed flutter suppression controller over
% % multiple airspeeds. It also shows that the interaction with pilot
% % commands is very low, since all initial transients are essentially
% % unaltered by the flutter suppression controller
% 
% figure(162)
% 
%     subplot(223)
%     step(multisys_openloop(2,1),multiCL(2,1),'g:',0.5);grid;
%     if exist('garyfyFigure','file');garyfyFigure;end;
%     ylabel('q [deg/s]');ylim([-20 10])
% 
%     subplot(221)
%     step(multisys_openloop(1,1),multiCL(1,1),'g:',0.5);grid;
%     if exist('garyfyFigure','file');garyfyFigure;end;
%     ylabel('theta [deg]')
%     legend('open loop','w/ flutter suppression','Location','Best')
% 
%     subplot(222)
%     step(multisys_openloop(3,1)*pi/180,multiCL(3,1)*pi/180,'g:',0.5);grid;
%     ylabel('az [m/s^2]');ylim([-1 6])
%     if exist('garyfyFigure','file');garyfyFigure;end;
% 
%     subplot(224)
%     step(multisys_openloop(4,1)*pi/180,multiCL(4,1)*pi/180,'g:',0.5);grid;
%     if exist('garyfyFigure','file');garyfyFigure;end;  
%     ylabel('acc [m/s^2]');ylim([-1 6])