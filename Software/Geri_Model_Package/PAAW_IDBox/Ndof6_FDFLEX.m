function [ndof6] = Ndof6_FDFLEX(AE6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata,InputType)
%% Develops the 6dof aeroelastic dynamic model for Geri,
%
% This is a BPD-modified version of the original from DKS. several
% parameters (i.e., aero coefficients) are now inputs to the function 
% aeroProp = structure of aerodynamic properities with fields:
%   .V (freestream velocity, ft/s), .rho (atmosheric pressure, lbs/ft2), .S
%   (characterisitc area, ft2), .b (span, ft), .cbar (chord, ft), alpha (AOA, rad), 
%   theta (trim pitch angle, rad).
% massProp = structure of mass properties with fields:
%   .m (mass, slugs), .Iyy (pitch inertia, slug-ft2), .Ixx (roll inertia,
%   slug-ft2), .Izz (yaw inertia, slug-ft2), omega (modal frequenies,
%   rad/s), zeta (modal damping).
%
% given the AE coefficients. 1000 ft altitude is assumed
% SS model built from the RB BFF model from the mAEWing1 VLM data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This code, written by David Schmidt, along with any acompanying data,
% is propietary to DK Schmidt and Associates. Please keep confidential.

% Copyright (c) 2016 by David K. Schmidt

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% In this version, the first six vibration modes are included (sym & antisym)

% States have been converted to u w theta q h beta phi p r eta 1-6 etadot 1-6
% Now body axes are being used instaed of stability axes

%   Units: Ft, slugs, rad, sec, 

if nargin<8 || isempty(InputType) 
    InputType = 'SymAsymInp'; %default are symmetric and anti-symetric inputs
end

m2f=3.28084; % meters to feet.
%k2sl=0.0685217659;  % kg to slugs
%r2d=180/3.14159;    % rad to deg
kt2fps=6080/3600;   % knots to fps
g = 32.17; %acceleration of gravity in ft/s^2

Vfps = Vmps*m2f; %aeroProp.V;
Vinf = Vfps/kt2fps;   % kts, change as necessary
AOA = aeroProp.alpha*180/pi();
% AOA = 2; % deg, estimated trim angle of attack used in RB model.
if isfield(aeroProp,'theta')
    THETA0 = aeroProp.theta;
else
    THETA0 = 0;               %this assumes flight path angle of -alpha (this results in the least difference from the old version)
%     THETA0 = aeroProp.alpha;  %this assumes 0 flight path angle
end
cth0 = cos(THETA0);
sth0 = sin(THETA0);

symins =  sum(coeffdata.IputIx < 6); %number of symmetric inputs

%% Mode shapes are in the "modes" structured array below, FEM and ModeShape must be in workspace
% The format is modes.X(i,j,k), where X = R (right) or L (left) wing,
% i= FEM VIBRATION mode number, j=FEM node span location, and k indicates plunge (1) or twist(2)
% 
% Ultimate units are FT and RADIANS, (orig FEM in inches) z down, theta wing leading-edge up.
% This data needed for modeling vehicle responses at sensor locations (C)

% Below - location of FEM nodes. Origin at vehicle nose, X aft, Y right wing span

XEAwing=[FEM.EA_points_coordinates(10,2);FEM.EA_points_coordinates(10:18,2)]/12; % Right or Left wing, data in in
XEAwing=(15.55/12)+XEAwing; % Adjustment for FEM origin to origin at nose of vehicle

% Mode 1

modes.R(1,:,1)=-[ModeShape.EA_modeshape7(10,4);ModeShape.EA_modeshape7(10:18,4)]/12; % Right wing Z in in to ft, sign convention
modes.L(1,:,1)=-[ModeShape.EA_modeshape7(1,4);ModeShape.EA_modeshape7(1:9,4)]/12; % Left wing Z in to ft
modes.R(1,:,2)=[ModeShape.EA_modeshape7(10,6);ModeShape.EA_modeshape7(10:18,6)]; % Right wing theta in rad
modes.L(1,:,2)=[ModeShape.EA_modeshape7(1,6);ModeShape.EA_modeshape7(1:9,6)]; % Left wing theta in rad
modes.R(1,:,3)=-[ModeShape.EA_modeshape7(10,5);ModeShape.EA_modeshape7(10:18,5)]; % Right wing phi in rad
modes.L(1,:,3)=-[ModeShape.EA_modeshape7(1,5);ModeShape.EA_modeshape7(1:9,5)]; % Left wing phi in rad

% Mode 2

modes.R(2,:,1)=-[ModeShape.EA_modeshape8(10,4);ModeShape.EA_modeshape8(10:18,4)]/12; % Right wing Z in in, sign convention
modes.L(2,:,1)=-[ModeShape.EA_modeshape8(1,4);ModeShape.EA_modeshape8(1:9,4)]/12; % Left wing Z in to ft
modes.R(2,:,2)=[ModeShape.EA_modeshape8(10,6);ModeShape.EA_modeshape8(10:18,6)]; % Right wing theta in rad
modes.L(2,:,2)=[ModeShape.EA_modeshape8(1,6);ModeShape.EA_modeshape8(1:9,6)]; % Left wing theta in rad
modes.R(2,:,3)=-[ModeShape.EA_modeshape8(10,5);ModeShape.EA_modeshape8(10:18,5)]; % Right wing phi in rad
modes.L(2,:,3)=-[ModeShape.EA_modeshape8(1,5);ModeShape.EA_modeshape8(1:9,5)]; % Left wing phi in rad

% Mode 3

modes.R(3,:,1)=-[ModeShape.EA_modeshape9(10,4);ModeShape.EA_modeshape9(10:18,4)]/12; % Right wing Z in in, sign convention
modes.L(3,:,1)=-[ModeShape.EA_modeshape9(1,4);ModeShape.EA_modeshape9(1:9,4)]/12; % Left wing Z in to ft
modes.R(3,:,2)=[ModeShape.EA_modeshape9(10,6);ModeShape.EA_modeshape9(10:18,6)]; % Right wing theta in rad
modes.L(3,:,2)=[ModeShape.EA_modeshape9(1,6);ModeShape.EA_modeshape9(1:9,6)]; % Left wing theta in rad
modes.R(3,:,3)=-[ModeShape.EA_modeshape9(10,5);ModeShape.EA_modeshape9(10:18,5)]; % Right wing phi in rad
modes.L(3,:,3)=-[ModeShape.EA_modeshape9(1,5);ModeShape.EA_modeshape9(1:9,5)]; % Left wing phi in rad

% Mode 4

modes.R(4,:,1)=-[ModeShape.EA_modeshape10(10,4);ModeShape.EA_modeshape10(10:18,4)]/12; % Right wing Z in in, sign convention
modes.L(4,:,1)=-[ModeShape.EA_modeshape10(1,4);ModeShape.EA_modeshape10(1:9,4)]/12; % Left wing Z in to ft
modes.R(4,:,2)=[ModeShape.EA_modeshape10(10,6);ModeShape.EA_modeshape10(10:18,6)]; % Right wing theta in rad
modes.L(4,:,2)=[ModeShape.EA_modeshape10(1,6);ModeShape.EA_modeshape10(1:9,6)]; % Left wing theta in rad
modes.R(4,:,3)=-[ModeShape.EA_modeshape10(10,5);ModeShape.EA_modeshape10(10:18,5)]; % Right wing phi in rad
modes.L(4,:,3)=-[ModeShape.EA_modeshape10(1,5);ModeShape.EA_modeshape10(1:9,5)]; % Left wing phi in rad

% 5th vibration mode

modes.R(5,:,1)=-[ModeShape.EA_modeshape11(10,4);ModeShape.EA_modeshape11(10:18,4)]/12; % Right wing Z in in, sign convention
modes.L(5,:,1)=-[ModeShape.EA_modeshape11(1,4);ModeShape.EA_modeshape11(1:9,4)]/12; % Left wing Z in to ft
modes.R(5,:,2)=[ModeShape.EA_modeshape11(10,6);ModeShape.EA_modeshape11(10:18,6)]; % Right wing theta in rad
modes.L(5,:,2)=[ModeShape.EA_modeshape11(1,6);ModeShape.EA_modeshape11(1:9,6)]; % Left wing theta in rad
modes.R(5,:,3)=-[ModeShape.EA_modeshape11(10,5);ModeShape.EA_modeshape11(10:18,5)]; % Right wing phi in rad
modes.L(5,:,3)=-[ModeShape.EA_modeshape11(1,5);ModeShape.EA_modeshape11(1:9,5)]; % Left wing phi in rad

% 6th vibration mode

modes.R(6,:,1)=-[ModeShape.EA_modeshape12(10,4);ModeShape.EA_modeshape12(10:18,4)]/12; % Right wing Z in in, sign convention
modes.L(6,:,1)=-[ModeShape.EA_modeshape12(1,4);ModeShape.EA_modeshape12(1:9,4)]/12; % Left wing Z in to ft
modes.R(6,:,2)=[ModeShape.EA_modeshape12(10,6);ModeShape.EA_modeshape12(10:18,6)]; % Right wing theta in rad
modes.L(6,:,2)=[ModeShape.EA_modeshape12(1,6);ModeShape.EA_modeshape12(1:9,6)]; % Left wing theta in rad
modes.R(6,:,3)=-[ModeShape.EA_modeshape12(10,5);ModeShape.EA_modeshape12(10:18,5)]; % Right wing phi in rad
modes.L(6,:,3)=-[ModeShape.EA_modeshape12(1,5);ModeShape.EA_modeshape12(1:9,5)]; % Left wing phi in rad

% Note that from VT FEM theta mode shapes + = LE up.
% Note that from FEM z mode shapes + = up

%% Geometry and Mass Properties for Geri

% S=1.085*m2f*m2f;
% cbar=0.39373*m2f;
% bw=10;
% W=13.78;
% m=W/32.17; % mass in slugs

% Iyy=1584.4/32.17/12/12; % Iyy sl-ft^2 Geri FEM 5.1
% Ixx=9021.6/32.17/12/12;Izz=9291.2/32.17/12/12;  % From VT FEM 5.1 Geri
% rho=0.0023081;

S = aeroProp.S; 
cbar = aeroProp.cbar; 
bw = aeroProp.b; 
m = massProp.m; 
W = m*g;

Iyy = massProp.Iyy;
Ixx = massProp.Ixx;
Izz = massProp.Izz;

rho = aeroProp.rho;

Vinf=Vinf*kt2fps;qS=0.5*rho*Vinf*Vinf*S;qSc=qS*cbar;qSb=qS*bw;

Xcg=23.73/12; 

k=12;  % Proper conversion for VT FEM, which has mass units of slug-in/ft
mgen(1:6)=1/k;
% omega(1:6) = ModeShape.natural_frequency(7:12)*2*pi;
% zeta(1:6) = 0.02;zeta(2) = 0.045;% NOTE: This damping increased to raise 2nd flutter speed to 61 kt

omega = massProp.omega;
zeta = massProp.zeta;

%% Load non-dimansional aeroelastic coefficients

AEClong=AE6.long;
AEClatdir=AE6.latdir;

CLeta=AEClong(1,:);CLetadotVinf=AEClong(2,:);
CMeta=AEClong(3,:);CMetadotVinf=AEClong(4,:);
CQw=AEClong(5,:)/Vinf;CQqVinf=AEClong(6,:);% changed alf to w
CQdele1=AEClong(7,:);CQdele2=AEClong(8,:);
CQdele3=AEClong(9,:);CQdele4=AEClong(10,:);
CQeta=AEClong(11:16,:);CQetadotVinf=AEClong(17:22,:);

CSeta=AEClatdir(1,:);CSetadotVinf=AEClatdir(2,:);
Cleta=AEClatdir(3,:);CletadotVinf=AEClatdir(4,:);
CNeta=AEClatdir(5,:);CNetadotVinf=AEClatdir(6,:);
CQbeta=AEClatdir(7,:);CQpVinf=AEClatdir(8,:);CQrVinf=AEClatdir(9,:);% Beta to v?
CQdela1=AEClatdir(10,:);CQdela2=AEClatdir(11,:);
CQdela3=AEClatdir(12,:);CQdela4=AEClatdir(13,:);

%replace certain coeffs with ones sent in via coeffdata ------------------
CLeta(coeffdata.flexI) = -coeffdata.CZ_eta_i;
CLetadotVinf(coeffdata.flexI) = -coeffdata.CZ_etadt_i;
CMeta(coeffdata.flexI) = coeffdata.Cm_eta_i;
CMetadotVinf(coeffdata.flexI) = coeffdata.Cm_etadt_i;
CQw(coeffdata.flexI) = coeffdata.CQ_alpha/Vinf;
CQqVinf(coeffdata.flexI) = coeffdata.CQ_q;
CQeta(coeffdata.flexI,coeffdata.flexI) = coeffdata.CQ_eta_i;
CQetadotVinf(coeffdata.flexI,coeffdata.flexI) = coeffdata.CQ_etadot_i;

if ~isempty(find(coeffdata.IputIx == 1, 1))
    CQdele1(coeffdata.flexI) = coeffdata.CQ_Li_long(:,coeffdata.IputIx == 1);
end
if ~isempty(find(coeffdata.IputIx == 2, 1))
    CQdele2(coeffdata.flexI) = coeffdata.CQ_Li_long(:,coeffdata.IputIx == 2);
end
if ~isempty(find(coeffdata.IputIx == 3, 1))
    CQdele3(coeffdata.flexI) = coeffdata.CQ_Li_long(:,coeffdata.IputIx == 3);
end
if ~isempty(find(coeffdata.IputIx == 4, 1))
    CQdele4(coeffdata.flexI) = coeffdata.CQ_Li_long(:,coeffdata.IputIx == 4);
end

CSeta(coeffdata.flexI) = coeffdata.CY_eta_i;
CSetadotVinf(coeffdata.flexI) = coeffdata.CY_etadt_i;
Cleta(coeffdata.flexI) = coeffdata.Cl_eta_i;
CletadotVinf(coeffdata.flexI) = coeffdata.Cl_etadt_i;
CNeta(coeffdata.flexI) = coeffdata.Cn_eta_i;
CNetadotVinf(coeffdata.flexI) = coeffdata.Cn_etadt_i;
CQbeta(coeffdata.flexI) = coeffdata.CQ_beta;
CQpVinf(coeffdata.flexI) = coeffdata.CQ_p;
CQrVinf(coeffdata.flexI) = coeffdata.CQ_r;

if ~isempty(find(coeffdata.IputIx == 6, 1))
    CQdela1(coeffdata.flexI) = coeffdata.CQ_Li_lat(:,find(coeffdata.IputIx == 6)-symins);
end
if ~isempty(find(coeffdata.IputIx == 7, 1))
    CQdela2(coeffdata.flexI) = coeffdata.CQ_Li_lat(:,find(coeffdata.IputIx == 7)-symins);
end
if ~isempty(find(coeffdata.IputIx == 8, 1))
    CQdela3(coeffdata.flexI) = coeffdata.CQ_Li_lat(:,find(coeffdata.IputIx == 8)-symins);
end
if ~isempty(find(coeffdata.IputIx == 9, 1))
    CQdela4(coeffdata.flexI) = coeffdata.CQ_Li_lat(:,find(coeffdata.IputIx == 9)-symins);
end

%% Calculate dimensional aeroelastic derivatives in array format

Zeta=-CLeta*qS/m;Zetadot=-CLetadotVinf*qS/m/Vinf;% 1x6 arrays 
Meta=CMeta*qSc/Iyy;Metadot=CMetadotVinf*qSc/Iyy/Vinf;
XIw=qSc*CQw./mgen;XIq=qSc*(CQqVinf./mgen)/Vinf;
XIdele1=qSc*CQdele1./mgen;XIdele2=qSc*CQdele2./mgen;
XIdele3=qSc*CQdele3./mgen;XIdele4=qSc*CQdele4./mgen;
XIeta(1:6,1:6)=0;
for i=1:6
    for j=1:6
        XIeta(i,j)=qSc*CQeta(i,j)/mgen(i);
    end
end
XIetadot(1:6,1:6)=0;
for i=1:6
    for j=1:6
        XIetadot(i,j)=qSc*CQetadotVinf(i,j)/mgen(i)/Vinf;
    end
end

Yeta=CSeta*qS/m;Yetadot=CSetadotVinf*qS/m/Vinf;
Leta=Cleta*qSb/Ixx;Letadot=CletadotVinf*qSb/Ixx/Vinf;
Neta=CNeta*qSb/Izz;Netadot=CNetadotVinf*qSb/Izz/Vinf;
XIbeta=qSb*CQbeta./mgen;XIp=qSb*(CQpVinf./mgen)/Vinf;XIr=qSb*(CQrVinf./mgen)/Vinf;% beta to v?
XIdela1=qSb*CQdela1./mgen;XIdela2=qSb*CQdela2./mgen;
XIdela3=qSb*CQdela3./mgen;XIdela4=qSb*CQdela4./mgen;

%% Rigid-body aero

%    This is the mAEWing1 data 
%     CLalf=4.59;
%     CLq=4.424;
%     CLdele1=0.794;
%     CLdele2=0.603;
%     CLdele3=0.506;
%     CLdele4=0.416;
%     CDalf=0.077;
%     CDq=0.073;
%     CDdele1=0.02;
%     CDdele2=0.0056;
%     CDdele3=0.0065;
%     CDdele4=0.0075;
%     CMalf=-0.1542;
%     CMq=-1.8685;
%     CMdele1=0.0224;
%     CMdele2=-0.0489;
%     CMdele3=-0.2013;
%     CMdele4=-0.3009; % cg = 23.74 in
    
    
%    This is the mAEWing1 data obtained from Tornado (FWD CG) ANTI-SYMMETRIC CONTROL INPUTS
%   NOTE: Tornado coefs based on aero coordinate frame, not body frame - need sign conversions
%     CSbeta=-0.210;
%     CSp=0.006;
%     CSr=0.073;
%     CSdela1=-0.0051;
%     CSdela2=-0.0038;
%     CSdela3=-0.0034;
%     CSdela4=-0.0010;
%     Clbeta=-0.0044;
%     Clp=-0.549;
%     Clr=0.011;
%     Cldela1=0.00401;
%     Cldela2=0.0975;
%     Cldela3=0.1395;
%     Cldela4=0.1568;
%     CNbeta=0.030;
%     CNp=-0.008;
%     CNr=-0.010;
%     CNdela1=0.0006;
%     CNdela2=0.0025;
%     CNdela3=0.0028;
%     CNdela4=0.0019;% New estimates for fwd cg
    
%load from coeffdata --------------------------------------------------
%Lift and drag are done below since coeffdata already has them in the body axes
CMalf = coeffdata.Cm_alpha;
CMq = coeffdata.Cm_q;

CSbeta = coeffdata.CY_beta;
CSp = coeffdata.CY_p;
CSr = coeffdata.CY_r;

Clbeta = coeffdata.Cl_beta;
Clp = coeffdata.Cl_p;
Clr = coeffdata.Cl_r;
CNbeta = coeffdata.Cn_beta;
CNp = coeffdata.Cn_p;
CNr = coeffdata.Cn_r;

%symmetric input coeffs
if ~isempty(find(coeffdata.IputIx == 1, 1))
    CLdele1 = -coeffdata.CZ_Li(coeffdata.IputIx == 1);
    CDdele1 = -coeffdata.CX_Li(coeffdata.IputIx == 1);
    CMdele1 = coeffdata.Cm_Li(coeffdata.IputIx == 1);
else
    CLdele1=0.794;
    CDdele1=0.02;
    CMdele1=0.0224;
end
if ~isempty(find(coeffdata.IputIx == 2, 1))
    CLdele2 = -coeffdata.CZ_Li(coeffdata.IputIx == 2);
    CDdele2 = -coeffdata.CX_Li(coeffdata.IputIx == 2);
    CMdele2 = coeffdata.Cm_Li(coeffdata.IputIx == 2);
else
    CLdele2=0.603;
    CDdele2=0.0056;
    CMdele2=-0.0489;
end
if ~isempty(find(coeffdata.IputIx == 3, 1))
    CLdele3 = -coeffdata.CZ_Li(coeffdata.IputIx == 3);
    CDdele3 = -coeffdata.CX_Li(coeffdata.IputIx == 3);
    CMdele3 = coeffdata.Cm_Li(coeffdata.IputIx == 3);
else
    CLdele3=0.506;
    CDdele3=0.0065;
    CMdele3=-0.2013;
end
if ~isempty(find(coeffdata.IputIx == 4, 1))
    CLdele4 = -coeffdata.CZ_Li(coeffdata.IputIx == 4);
    CDdele4 = -coeffdata.CX_Li(coeffdata.IputIx == 4);
    CMdele4 = coeffdata.Cm_Li(coeffdata.IputIx == 4);
else
    CLdele4=0.416;
    CDdele4=0.0075;
    CMdele4=-0.3009;
end

%asymmetric input coeffs
if ~isempty(find(coeffdata.IputIx == 6, 1))
    CSdela1 = coeffdata.CY_Li(find(coeffdata.IputIx == 6)-symins);
    Cldela1 = coeffdata.Cl_Li(find(coeffdata.IputIx == 6)-symins);
    CNdela1 = coeffdata.Cn_Li(find(coeffdata.IputIx == 6)-symins);
else
    CSdela1=-0.0051;
    Cldela1=0.00401;
    CNdela1=0.0006;
end
if ~isempty(find(coeffdata.IputIx == 7, 1))
    CSdela2 = coeffdata.CY_Li(find(coeffdata.IputIx == 7)-symins);
    Cldela2 = coeffdata.Cl_Li(find(coeffdata.IputIx == 7)-symins);
    CNdela2 = coeffdata.Cn_Li(find(coeffdata.IputIx == 7)-symins);
else
    CSdela2=-0.0038;
    Cldela2=0.0975;
    CNdela2=0.0025;
end
if ~isempty(find(coeffdata.IputIx == 8, 1))
    CSdela3 = coeffdata.CY_Li(find(coeffdata.IputIx == 8)-symins);
    Cldela3 = coeffdata.Cl_Li(find(coeffdata.IputIx == 8)-symins);
    CNdela3 = coeffdata.Cn_Li(find(coeffdata.IputIx == 8)-symins);
else
    CSdela3=-0.0034;
    Cldela3=0.1395;
    CNdela3=0.0028;
end
if ~isempty(find(coeffdata.IputIx == 9, 1))
    CSdela4 = coeffdata.CY_Li(find(coeffdata.IputIx == 9)-symins);
    Cldela4 = coeffdata.Cl_Li(find(coeffdata.IputIx == 9)-symins);
    CNdela4 = coeffdata.Cn_Li(find(coeffdata.IputIx == 9)-symins);
else
    CSdela4=-0.0010;
    Cldela4=0.1568;
    CNdela4=0.0019;
end
    
%%   Build SS Rigid-Body Model

%     U0=Vinf*cos(AOA/57.3);          % needed for conversion to body axes
%     W0=Vinf*sin(AOA/57.3);
    
    U0=Vinf*cos(AOA*pi()/180);          % needed for conversion to body axes
    W0=Vinf*sin(AOA*pi()/180);
    
    c2U0=cbar/2/Vinf;                      % Convert non-dimensional derivs
%     CLq=CLq*c2U0;
    CMq=CMq*c2U0;
%     CDq=CDq*c2U0;
    CLtrim=W/qS;
    
    b2U0=bw/2/Vinf;                      % Convert non-dimensional derivs
    CSp=CSp*b2U0;CSr=CSr*b2U0;CNp=CNp*b2U0;CNr=CNr*b2U0;Clp=Clp*b2U0;Clr=Clr*b2U0;
    
%     CZalf=-CLalf*cos(AOA/57.3)-CDalf*sin(AOA/57.3);% Convert to body axes
%     CZq=-CLq*cos(AOA/57.3)-CDq*sin(AOA/57.3);%    NOTE: in body axes alfnot = wdot/Vinf
    
    CZalf = coeffdata.CZ_alpha;
    CZq = coeffdata.CZ_q*c2U0;
    
%     CXalf=-CDalf*cos(AOA/57.3)+CLalf*sin(AOA/57.3);
%     CXq=-CDq*cos(AOA/57.3)+CLq*sin(AOA/57.3); % add CZdels and CXdels

    CXalf = coeffdata.CX_alpha;
    CXq = coeffdata.CX_q*c2U0;
    
    Mw=(CMalf/Vinf)*qSc/Iyy;Mq=CMq*qSc/Iyy;Mu=0;Malfdot=0;% changed alf to w
    Mdele1=CMdele1*qSc/Iyy;Mdele2=CMdele2*qSc/Iyy;Mdele3=CMdele3*qSc/Iyy;Mdele4=CMdele4*qSc/Iyy;
    Xw=((CXalf)+CLtrim)*qS/(m*Vinf);Xalfdot=0;Xq=CXq*qS/m;Xu=0;% Converted to body axes
    Xdele1=-CDdele1*qS/m;Xdele2=-CDdele2*qS/m;Xdele3=-CDdele3*qS/m;Xdele4=-CDdele4*qS/m;
    Zw=(CZalf/Vinf)*qS/m;Zalfdot=0;Zq=CZq*qS/m;Zu=-(2*CLtrim/Vinf)*qS/m;% Alf derivs Converted to body axes
    Zdele1=-CLdele1*qS/m;Zdele2=-CLdele2*qS/m;Zdele3=-CLdele3*qS/m;Zdele4=-CLdele4*qS/m;
    
    Lbeta=Clbeta*qSb/Ixx;Lp=Clp*qSb/Ixx;Lr=Clr*qSb/Ixx;
    Ldela1=Cldela1*qSb/Ixx;Ldela2=Cldela2*qSb/Ixx;Ldela3=Cldela3*qSb/Ixx;Ldela4=Cldela4*qSb/Ixx;
    Ybeta=CSbeta*qS/m;Yp=CSp*qS/m;Yr=CSr*qS/m;
    Ydela1=CSdela1*qS/m;Ydela2=CSdela2*qS/m;Ydela3=CSdela3*qS/m;Ydela4=CSdela4*qS/m;
    Nbeta=CNbeta*qSb/Izz;Nr=CNr*qSb/Izz;Np=CNp*qSb/Izz;
    Ndela1=CNdela1*qSb/Izz;Ndela2=CNdela2*qSb/Izz;Ndela3=CNdela3*qSb/Izz;Ndela4=CNdela4*qSb/Izz;

    % Put in form Mxdot=Ax + Bu, -> xdot=MinvAx + MinvBu

    M=[1 -Xalfdot/Vinf 0 0 0;0 (1-Zalfdot/Vinf) 0 0 0;0 0 1 0 0;...
        0 -Malfdot/Vinf 0 1 0;0 0 0 0 1];

    % longitudinal
    % MODIFIED FOR SHALLOW CLUMB AND DECENT REFERENCE CONDITIONS 8/17/2017
    % STILL ASSUMING PHI_REF) = 0
%     a=[Xu Xw -32.17 (Xq-W0) 0;Zu Zw 0 (U0+Zq) 0;% add h and change alf to w in EOMs
%     0 0 0  1 0;Mu Mw 0 Mq 0;0 -1 Vinf 0 0];
    a=[Xu Xw -g*cth0 (Xq-W0) 0;Zu Zw -g*sth0 (U0+Zq) 0;% With GAMMA0 not = 0
    0 0 0  1 0;Mu Mw 0 Mq 0;sth0 -cth0 (U0*cth0+W0*sth0) 0 0];
    b=[Xdele1 Xdele2 Xdele3 Xdele4;
        Zdele1 Zdele2 Zdele3 Zdele4;
        0 0 0 0;Mdele1 Mdele2 Mdele3 Mdele4;0 0 0 0];
    b(:,5)=[1/m;0;0;-1/(12*Iyy);0];  % add thrust input NOTE ******* 7/16
    along=M\a;
    blong=M\b;
    clong=eye(5);clong(2,2)=1/U0;%d=zeros(5,5); d not used %OUTPUT is alpha, not w!!
    
% Long states are u, w, theta, q h; inputs are Sym flaps 1-4 and T, Units, ft, rad, sec
%                           NOTE state and controls changes
% Add'l states are etai and etaidot, i=1-6
    
    % larDir
    % MODIFIED FOR SHALLOW CLUMB AND DECENT REFERENCE CONDITIONS 8/17/2017
    % STILL ASSUMING PHI) = 0
%     alatdir=[Ybeta/Vinf 32.2/Vinf Yp/Vinf ((Yr/Vinf)-1);0 0 1 0;
%          Lbeta 0 Lp  Lr;Nbeta 0 Np Nr];
    alatdir=[Ybeta/Vinf (g*cth0)/Vinf (Yp+W0)/Vinf (Yr-U0)/Vinf;0 0 1 0;
         Lbeta 0 Lp  Lr;Nbeta 0 Np Nr];
    blatdir=[Ydela1/Vinf Ydela2/Vinf Ydela3/Vinf Ydela4/Vinf;
         0 0 0 0;Ldela1 Ldela2 Ldela3 Ldela4;
        Ndela1 Ndela2 Ndela3 Ndela4];
    clatdir=eye(4);%dlatdir=zeros(4,4);% beta to v conversions?
    
% Latdir states are beta, phi, p, r; inputs are AS flaps 1-4 and T, Units, ft, rad, sec
%                           NOTE state and controls changes
% Add'l states are etai and etaidot, i=1-6 

a6=[along,zeros(5,4);zeros(4,5),alatdir];
b6=[blong,zeros(5,4);zeros(4,5),blatdir];
c6=[clong,zeros(5,4);zeros(4,5),clatdir];

%% Build (21x21) ndof state model with 6 RB DOFs and 6 vib modes
% elastic states are etas then etadots, to use array coefficients
% u w theta q h beta phi p r eta 1-6 etadot 1-6

andof=[a6,zeros(9,12);zeros(12,21)];
andof(2,10:21)=[Zeta,Zetadot];
andof(4,10:21)=[Meta,Metadot];
andof(6,10:21)=[Yeta/Vinf,Yetadot/Vinf];
andof(8,10:21)=[Leta,Letadot];
andof(9,10:21)=[Neta,Netadot];
andof(10:15,16:21)=eye(6);
OMEGA=diag(omega.*omega);
ZETOMEG=diag(2*(zeta.*omega));
andof(16:21,:)=[zeros(6,1),XIw',zeros(6,1),XIq',zeros(6,1),XIbeta',zeros(6,1)...
        XIp',XIr',XIeta,XIetadot];
andof(16:21,10:15)=andof(16:21,10:15)-OMEGA;
andof(16:21,16:21)=andof(16:21,16:21)-ZETOMEG;

bndof=zeros(21,10); % modified for new thrust input
bndof(1:9,1:9)=b6;
bndof(16:21,1:4)=[XIdele1',XIdele2',XIdele3',XIdele4'];
bndof(16:21,6:9)=[XIdela1',XIdela2',XIdela3',XIdela4'];
bndof(:,10)=andof(:,2);bndof(5,10)=0; % add wgust input too

cndof=[c6,zeros(9,12)];dndof=zeros(9,10);

%% Add add'l responses to c and d matrices (NEW STATE DEFINITIONS)

% Z Mode Shapes at Tip Acel Sensor locations

modes.TIPS(1:6,1:4,1:2)=0;
modes.TIPS(1,1:4,1)=-ModeShape.EA_modeshape7(28:31,4)/12;
modes.TIPS(2,1:4,1)=-ModeShape.EA_modeshape8(28:31,4)/12;
modes.TIPS(3,1:4,1)=-ModeShape.EA_modeshape9(28:31,4)/12;
modes.TIPS(4,1:4,1)=-ModeShape.EA_modeshape10(28:31,4)/12;
modes.TIPS(5,1:4,1)=-ModeShape.EA_modeshape11(28:31,4)/12;
modes.TIPS(6,1:4,1)=-ModeShape.EA_modeshape12(28:31,4)/12;

cndof(10,:)=[0,0,0,1,0,zeros(1,4),zeros(1,6),modes.R(:,1,2)'];dndof(10,:)=zeros(1,10);% Pitch rate on CL

cndof(11,:)=[zeros(1,5),0,0,1,0,zeros(1,6),0.5*(modes.R(:,1,3)+modes.L(:,1,3))'];dndof(11,:)=zeros(1,10); % Roll rate on CL

xacg=Xcg-XEAwing(1); % Distance aft from node 1 to cg, for accel @ cg.- ft
zaccel=[0,1,-Vinf,0,0,zeros(1,4),zeros(1,6),(modes.R(:,1,1)+xacg*modes.R(:,1,2))'];
cndof(12,:)=zaccel*andof;dndof(12,:)=zaccel*bndof; % z accel at cg

DelXaft=22.91/12-Xcg;  % aft accelerometer located on CL DelXaft aft of cg, ft
xaaft=22.91/12-XEAwing(1);  % distance aft on CL from CL-node to aft accel, ft
zaccelr=[0,1,-Vinf,DelXaft,0,zeros(1,4),zeros(1,6),(modes.R(:,1,1)+xaaft*modes.R(:,1,2))'];
cndof(13,:)=zaccelr*andof;dndof(13,:)=zaccelr*bndof; % z accel - aft accelerometer on CL

DelXaft=3.19/12-Xcg;  % fwd accelerometer located on CL -DelXaft fwd of cg, ft
xafwd=3.19/12-XEAwing(1);  % distance fwd (negative) on CL from CL-node to fwd accel, ft
zaccelf=[0,1,-Vinf,DelXaft,0,zeros(1,4),zeros(1,6),(modes.R(:,1,1)+xafwd*modes.R(:,1,2))'];    
cndof(14,:)=zaccelf*andof;dndof(14,:)=zaccelf*bndof; % z accel - fwd accelerometer on CL

DelXaft=(15.55+FEM.EA_points_coordinates(31,2))/12-Xcg;  % distance aft from cg to aft right accel, ft
Yaccel=FEM.EA_points_coordinates(31,3)/12;   % span location of accel - ft
zaccelrr=[0,1,-Vinf,DelXaft,0,0,0,Yaccel,0,zeros(1,6),modes.TIPS(:,4,1)'];   % accel at accel node 9006, Right wing tip
cndof(15,:)=zaccelrr*andof;dndof(15,:)=zaccelrr*bndof; % z accel, RIGHT wing aft.

zaccellr=[0,1,-Vinf,DelXaft,0,0,0,-Yaccel,0,zeros(1,6),modes.TIPS(:,2,1)'];   % accel at accel node 9004, Left wing tip
cndof(16,:)=zaccellr*andof;dndof(16,:)=zaccellr*bndof; % z accel, LEFT wing aft.

DelXaft=(15.55+FEM.EA_points_coordinates(30,2))/12-Xcg;  % distance aft from cg to fwd right accel, ft
zaccelrf=[0,1,-Vinf,DelXaft,0,0,0,Yaccel,0,zeros(1,6),modes.TIPS(:,3,1)'];   % accel at accel node 9006, Right wing tip
cndof(17,:)=zaccelrf*andof;dndof(17,:)=zaccelrf*bndof; % z accel, RIGHT wing, fwd

zaccellf=[0,1,-Vinf,DelXaft,0,0,0,-Yaccel,0,zeros(1,6),modes.TIPS(:,1,1)'];   % accel at accel node 9006, Right wing tip
cndof(18,:)=zaccellf*andof;dndof(18,:)=zaccellf*bndof; % z accel, LEFT wing, fwd

cndof(19:27,:)=zeros(9,21);dndof(19:27,1:9)=eye(9);  % control-surface and thrust deflections

cndof(28,:)=zeros(1,21);
cndof(28,2:3)=[-1 Vinf];dndof(28,:)=zeros(1,10); % add hdot to measurements

% Add measurements for different accel blendings

cndof(29,:)=cndof(15,:)-cndof(17,:);dndof(29,:)=dndof(15,:)-dndof(17,:);% Right twist; RA-RF Tips
cndof(30,:)=cndof(16,:)-cndof(18,:);dndof(30,:)=dndof(16,:)-dndof(18,:);% Left twist: LA-LF Tips
cndof(31,:)=cndof(29,:)-cndof(30,:);dndof(31,:)=dndof(29,:)-dndof(30,:);% Asym twist: Rtwist-Ltwist


%% SS system model

%eig(andof)

%added units (BPD: 5/21/2018)
ndof6=ss(andof,bndof,cndof,dndof);
ndof6.StateName={'u','w','theta','q','h','beta','phi','p','r','eta1','eta2','eta3','eta4','eta5','eta6',...
    'eta1dt','eta2dt','eta3dt','eta4dt','eta5dt','eta6dt'};
ndof6.StateUnit={'ft/s','ft/s','rad','rad/s','ft','rad','rad','rad/s','rad/s','-','-','-','-','-',...
    '-','1/s','1/s','1/s','1/s','1/s','1/s'};
ndof6.InputName={'Sym BF','Sym IB','Sym Mid','Sym OB','Thrust','AS BF','AS IB','AS Mid','AS Ob','wGust'};
ndof6.InputUnit={'rad','rad','rad','rad','lb','rad','rad','rad','rad','ft/s'};
ndof6.OutputName={'u','alpha','theta','q','h','beta','phi','p','r','qcg','pcg','nzcg','nzCBaft',...
    'nzCBfwd','nzRwingaft','nzLwingaft','nzRwingfwd','nzLwingfwd','delBFS','delIBS','delmidS',...
    'delOBS','delT lb','delBFAS','delIBAS','delmidAS','delOBAS','hdot','Right Twist','Left Twist',...
    'Asym Twist'}; %changed w to alpha (8/1/2017)
ndof6.OutputUnit={'ft/s','rad','rad','rad/s','ft','rad','rad','rad/s','rad/s','rad/s','rad/s',...
    'ft/s^2','ft/s^2','ft/s^2','ft/s^2','ft/s^2','ft/s^2','ft/s^2','rad','rad','rad','rad','lb',...
    'rad','rad','rad','rad','ft/s','ft/s^2','ft/s^2','ft/s^2'};

%change input type (re-allocate) to direct inputs if indicated
if strcmp(InputType,'DirectInp')
           
    Tinpinv = [1 0 0 0 0  1  0  0  0 0;
               1 0 0 0 0 -1  0  0  0 0;
               0 1 0 0 0  0  1  0  0 0;
               0 1 0 0 0  0 -1  0  0 0;
               0 0 1 0 0  0  0  1  0 0;
               0 0 1 0 0  0  0 -1  0 0;
               0 0 0 1 0  0  0  0  1 0;
               0 0 0 1 0  0  0  0 -1 0;
               0 0 0 0 1  0  0  0  0 0;
               0 0 0 0 0  0  0  0  0 1];
           
    Tinp = ss(inv(Tinpinv));
    Tinp.inputname = {'L1','R1','L2','R2','L3','R3','L4','R4','Thrust','wGust'};
    Tinp.inputunit = {'rad','rad','rad','rad','rad','rad','rad','rad','lb','ft/s'};
    ndof6 = ndof6*Tinp;
    
end

end
