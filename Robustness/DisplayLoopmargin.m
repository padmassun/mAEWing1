function [ICM_G, ICM_P, ICM_D, IDM_G, IDM_P, OCM_G, OCM_P, OCM_D, ODM_G, ODM_P, MMI_G, MMI_P, MMO_G, MMO_P, MMIO_G, MMIO_P] = DisplayLoopmargin(P,C)
% [ICM_G, ICM_P, ICM_D, IDM_G, IDM_P, OCM_G, OCM_P, OCM_D, ODM_G, ...
%  ODM_P, MMI_G, MMI_P, MMO_G, MMO_P, MMIO_G, MMIO_P] = DisplayLoopmargin(P,C)
% Calls LOOPMARGIN and outputs the results in a structured way.
%
% ICM_G : minimum classical gain margin at input
% ICM_P : minimum classical phase margin at input
% ICM_D : minimum classical delay margin at input
% IDM_G : symmetric disk gain margin at input
% IDM_P : symmetric disk phase margin at input
% 
% OCM_G : minimum classical gain margin at output
% OCM_P : minimum classical phase margin at output
% OCM_D : minimum classical delay margin at output
% ODM_G : symmetric disk gain margin at output
% ODM_P : symmetric disk gain margin at output
% 
% MMI_G  : symmetric multiloop input disk gain margin
% MMI_P  : symmetric multiloop input disk phase margin
% MMO_G  : symmetric multiloop output disk gain margin 
% MMO_P  : symmetric multiloop output disk phase margin
% MMIO_G : symmetric multiloop IO disk gain margin
% MMIO_P : symmetric multiloop IO disk phase margin


[ny, nu] = size(P);
[CMI,DMI,MMI,CMO,DMO,MMO,MMIO] = loopmargin(P,C);

%
if CMI(1).Stable
    fprintf('Closed-loop system is stable.\n\n')
else
    fprintf('Closed-loop system is UNSTABLE.\n\n')
    ICM_G = 0;
    ICM_P = 0;
    ICM_D = 0;
    IDM_G = 0;
    IDM_P = 0;

    OCM_G = 0;
    OCM_P = 0;
    OCM_D = 0;
    ODM_G = 0;
    ODM_P = 0;

    MMI_G  = 0;
    MMI_P  = 0;
    MMO_G  = 0;
    MMO_P  = 0;
    MMIO_G = 0;
    MMIO_P = 0;
    return
end



% Display Input Margins
for ii=1:nu
    if isempty(CMI(ii).PhaseMargin); CMI(ii).PhaseMargin=inf; end
    if isempty(CMI(ii).DelayMargin); CMI(ii).DelayMargin=inf; end
    if isempty(CMI(ii).GainMargin); CMI(ii).GainMargin=inf; end
    
    [minCMI_P(ii), minCMI_P_idx] = min(abs(CMI(ii).PhaseMargin));
    [minCMI_D(ii), minCMI_D_idx] = min(CMI(ii).DelayMargin);
    [~, minCMI_G_idx] = min(abs(CMI(ii).GainMargin));
    minCMI_G(ii) = CMI(ii).GainMargin(minCMI_G_idx);
    DMI_P(ii) = DMI(ii).PhaseMargin(2);
    DMI_G(ii) = DMI(ii).GainMargin(2);
    
    fprintf(['Breaking Loop at Input ' int2str(ii) ' ("' P.InputName{ii} '")\n']) 
    if ~isfinite(minCMI_G(ii))
        fprintf('\t Minimum Gain Margin: \t Infinite\n')
    else
        fprintf('\t Minimum Gain Margin: \t %4.1f dB \t\t\t\t\t at %3.1f rad/s\n',mag2db(minCMI_G(ii)),CMI(ii).GMFrequency(minCMI_G_idx))
    end
    if ~isfinite(minCMI_P(ii))
        fprintf('\t Minimum Phase Margin: \t Infinite\n')
    else
        fprintf('\t Minimum Phase Margin: \t %4.1f degrees \t\t\t\t at %3.1f rad/s\n',minCMI_P(ii),CMI(ii).PMFrequency(minCMI_P_idx))
    end
    if ~isfinite(minCMI_D(ii))
        fprintf('\t Minimum Delay Margin: \t Infinite\n')
    else
    fprintf('\t Minimum Delay Margin: \t %4.1f milliseconds \t\t\t at %3.1f rad/s\n',minCMI_D(ii)*1000,CMI(ii).DMFrequency(minCMI_D_idx))
    end
    fprintf('\t Symmetric Disk Margin: \t %4.1f degrees / %2.1f dB \t at %3.1f rad/s\n',DMI_P(ii),mag2db(DMI_G(ii)),DMI(ii).Frequency)
end

% Display Output Margins
for ii=1:ny
    
    if isempty(CMO(ii).PhaseMargin); CMO(ii).PhaseMargin=inf; end
    if isempty(CMO(ii).DelayMargin); CMO(ii).DelayMargin=inf; end
    if isempty(CMO(ii).GainMargin); CMO(ii).GainMargin=inf; end
    
    [minCMO_P(ii), minCMO_P_idx] = min(abs(CMO(ii).PhaseMargin));
    [minCMO_D(ii), minCMO_D_idx] = min(CMO(ii).DelayMargin);
    [~, minCMO_G_idx] = min(abs(CMO(ii).GainMargin));
    minCMO_G(ii) = CMO(ii).GainMargin(minCMO_G_idx);
    DMO_P(ii) = DMO(ii).PhaseMargin(2);
    DMO_G(ii) = DMO(ii).GainMargin(2);
    
    
    fprintf(['Breaking Loop at Output ' int2str(ii) ' ("' P.OutputName{ii} '")\n'])
    if ~isfinite(minCMO_G(ii))
        fprintf('\t Minimum Gain Margin: \t Infinite\n')
    else
        fprintf('\t Minimum Gain Margin: \t %4.1f dB \t\t\t\t\t at %3.1f rad/s\n',mag2db(minCMO_G(ii)),CMO(ii).GMFrequency(minCMO_G_idx))
    end
    if ~isfinite(minCMO_P(ii))
        fprintf('\t Minimum Phase Margin: \t Infinite\n')
    else
        fprintf('\t Minimum Phase Margin: \t %4.1f degrees \t\t\t\t at %3.1f rad/s\n',minCMO_P(ii),CMO(ii).PMFrequency(minCMO_P_idx))
    end
    if ~isfinite(minCMO_D(ii))
        fprintf('\t Minimum Delay Margin: \t Infinite\n')
    else
    fprintf('\t Minimum Delay Margin: \t %4.1f milliseconds \t\t\t at %3.1f rad/s\n',minCMO_D(ii)*1000,CMO(ii).DMFrequency(minCMO_D_idx))
    end
    fprintf('\t Symmetric Disk Margin: \t %4.1f degrees / %2.1f dB \t at %3.1f rad/s\n',DMO_P(ii),mag2db(DMO_G(ii)),DMO(ii).Frequency)
end

% Display MultiInput/MultiOutput Margins


fprintf('Simultaneous Perturbations at Inputs and Outputs\n')
    fprintf('\t Multi Input Disk Margin: \t %2.2f degrees / %2.2f dB \t at %3.1f rad/s\n',MMI.PhaseMargin(2),db(MMI.GainMargin(2)),MMI.Frequency)
    fprintf('\t Multi Output Disk Margin: \t %2.2f degrees / %2.2f dB \t at %3.1f rad/s\n',MMO.PhaseMargin(2),db(MMO.GainMargin(2)),MMO.Frequency)
    fprintf('\t Multi IO Disk Margin: \t\t %2.2f degrees / %2.2f dB \t at %3.1f rad/s\n',MMIO.PhaseMargin(2),db(MMIO.GainMargin(2)),MMIO.Frequency)
    %fprintf('\t Gap Metric Margin: \t\t\t\t\t %2.2f \t\t\t at %3.1f rad/s\n',NCFMARG,NCFFREQ)

ICM_G = minCMI_G;
ICM_P = minCMI_P;
ICM_D = minCMI_D;
IDM_G = DMI_G;
IDM_P = DMI_P;

OCM_G = minCMO_G;
OCM_P = minCMO_P;
OCM_D = minCMO_D;
ODM_G = DMO_G;
ODM_P = DMO_P;

MMI_G  = MMI.GainMargin(2);
MMI_P  = MMI.PhaseMargin(2);
MMO_G  = MMO.GainMargin(2);
MMO_P  = MMO.PhaseMargin(2);
MMIO_G = MMIO.GainMargin(2);
MMIO_P = MMIO.PhaseMargin(2);