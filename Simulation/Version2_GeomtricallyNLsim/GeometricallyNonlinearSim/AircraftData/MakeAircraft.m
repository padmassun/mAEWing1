%%MakeAircraft  Generates structural and aerodynamic models for simulation
%
%   Calls FEM, VLM and DLM code and saves the information needed for
%   simulation with 6DOF nonlinear mechanics, 6DOF linear FEM based
%   structural model and 6DOF geometrically non-linear aerodynamics.

%% Clear workspace
clear
close all
clc

%% Define parameters
% Select aircraft
AircraftID = 'Geri';

% Select if figuers are displayed
DisplayFig = false;

% Select aerodynamic type
%   Aeroypte = 'VLM' for only steady aerodynamic model
%   Aeroypte = 'DLM' for steady and unsteady aerodynamic model
AeroType = 'DLM';

% Display aerodynamic model choice
if strcmp(AeroType,'VLM')
    fprintf('Steady aerodynamic model chosen\n')
elseif strcmp(AeroType,'DLM')
    fprintf('Unsteady aerodynamic model chosen\n')
    addpath('DLMCode/')
else
    error('AeroTYpe should be "VLM" or "DLM"')
end

%% Aircraft configuration

% Configure the aircraft
AircraftConfig = ConfigAircraft(AircraftID);


% Data used from aircraft configutaion 
    % Mean Aerodynamic Chord (reference length)
    MAC = AircraftConfig.MAC;

    % Structural Model details
    StrucModelFile = AircraftConfig.StrucModelFile; 

    % Script to obtain aerodynamic grid
    GriddingScript = AircraftConfig.GriddingScript; 

    % Script to obtain spline grid
    SplineScript = AircraftConfig.SplineScript;

%% Structural model

% Load the structural model
load(StrucModelFile);

% Data used from structural model
    % Coordinates of FEM nodcdes 
    %   (See FEM code for details about the coordinates system)
    Coords_FEM = geom.strcgrid.coords;

    % Position vector of the structural node corresponing to the CG
    Rcg = modes.rcg;                        

%% Obtain aerodynamic grid

% Run gridding script
griddata = feval(GriddingScript);

% Data used from the grid
NumPanels = griddata.NumPanels;         % Number of aerodynamic panell
Aerogrid = griddata.Aerogrid;           % Centroid of aerodynamic panels
BodySpan = griddata.BodySpan;           % Span of the center body
PanelSpan = griddata.PanelSpan;         % Span of each panel
PanelLength = griddata.PanelLength;     % Flowwise length of each panel
PanelIndexCS = griddata.PanelIndexCS;   % Index of control surface panels
CSVertices = griddata.CSVertices;       % Control surface vertices
PanelData = griddata.PanelData;         % PanelData
NodeData = griddata.NodeData;           % NodeData
Nhat0 = griddata.Nhat0;                 % Normal vector of each panel
CSHingeDC = griddata.CSHingeDC;         % CS hinge direction cosine
CSPanelIndex = griddata.CSPanelIndex;   % Index of each CS panels
CSPanelIndex_all = griddata.CSPanelIndex_all;   % Index of all CS panels

%% Obtain spline grid
[Coords_spline,Connections] = SplineScript(Coords_FEM,BodySpan); 

%% Transformation matrices

% Structural to spline 
Tsf = calc_Tsf(Coords_FEM,Coords_spline,Connections);   

% Spline to aerodynamic
Tas = calc_Tas(Coords_spline,Aerogrid);

% Structural to aerodynamic
Taf = Tas*Tsf;                  

%% Obtain flexible mode shapes

% Flexible mode shapes in aerodynamic grid
Phif_aero = Taf*modes.Phif;     

% Number of flexible modes
NumfModes = size(Phif_aero,2);  

% Display flexible modes
% NOTE: The flexible modes are scaled for better visualization. This
% scaling does not affect the calculated modes
if DisplayFig
    flexfigscale = 2;
    for i = 1:size(Phif_aero,2)
        figure
        plot3(Aerogrid(1,:),Aerogrid(2,:),Aerogrid(3,:),'b*')
        hold on
        plot3(Aerogrid(1,:)+flexfigscale*Phif_aero(1:6:end,i)',...
                Aerogrid(2,:)+flexfigscale*Phif_aero(2:6:end,i)', ...
                Aerogrid(3,:)+flexfigscale*Phif_aero(3:6:end,i)','r*')
        xlabel('x'); ylabel('y'); zlabel('z'); grid on;
        legend('Undeflected nodes','Deflected nodes')
        axis equal
        view(110,60)
        title(['Aerodynic grid flexible mode: ', num2str(i)])
    end
end
%             
%% Obtain panel direction dependency on flexible deflection
Nhat_eta = getNhateta(Nhat0,Phif_aero,NumPanels,NumfModes);

%% Obtain steady aerodynamic model
VLMData = getSteadyAero(NumPanels,PanelData,NodeData,Nhat0,Nhat_eta,Rcg);
D0 = VLMData.D0;

%% If needed, obtain the unsteady aerodynamic model
if strcmp(AeroType,'DLM')
    DLMData = getUnsteadyAero(NumPanels,PanelData,NodeData, ...
                                       PanelSpan,PanelLength,MAC,D0);
end

%% Save AircraftData for Simulation
AircraftData.Aero.VLMData = VLMData;
AircraftData.Aero.NumPanels = NumPanels;
AircraftData.Aero.Aerogrid = Aerogrid;
AircraftData.Aero.Nhat0 = Nhat0;
AircraftData.Aero.Nhat_eta = Nhat_eta;
AircraftData.Aero.MAC = MAC;
if strcmp(AeroType,'DLM')
    AircraftData.Aero.DLMData = DLMData;
end

AircraftData.CS.CSTau = 300;
AircraftData.CS.CSHingeDC = CSHingeDC;
AircraftData.CS.CSPanelIndex = CSPanelIndex;
AircraftData.CS.CSPanelIndex_all = CSPanelIndex_all;

AircraftData.MassProp.mass = modes.Mass;
AircraftData.MassProp.inertia = modes.Inertia;

AircraftData.ModalStruc.InvMass = 1./modes.Mf;
AircraftData.ModalStruc.Damping = modes.Bf;
AircraftData.ModalStruc.Stiffness = modes.Kf;
AircraftData.ModalStruc.Phif_aero = Phif_aero;

save(['AircraftData_',AircraftID],'AircraftData')