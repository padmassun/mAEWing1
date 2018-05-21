function coeffsout = getcoeffsFDmod(sysePID0,massProp,aeroProp,flexI)
% inputs: sysePID0 - model identified using Dave Schmidt's model as
%          reference
% 
%         massProp: contains mass properties of aircraft including natural frequencies
%
%         aeroProp: structue with aero properties (V, S, cbar, etc.)
%
%          flexI: the structural states to be included; if ith, jth and kth
%          modes are to be included, flexI = [i j k]
[A,B,C,D] = ssdata(sysePID0);
 
% V = 23.2*3.28;  % airspeed ft/s
% rho = 0.0023081;%1.175;  % air density at 400 m
% q = 0.5*rho*V^2;
% 
% S = 1.085*3.28*3.28; % all lengths in m
% b = 10;
% cbar = 0.39*3.28;

V = aeroProp.V; % airspeed ft/s
rho = aeroProp.rho; % air density at 400 m
q = 0.5*rho*V^2;

if isfield(aeroProp,'alpha')
   alpha = aeroProp.alpha;
else
    alpha = 0;
end

W = V*sin(alpha);
U = V*cos(alpha);

S = aeroProp.S;  % characteristic area in ft^2
b = aeroProp.b;  % wingspan in ft
cbar = aeroProp.cbar;  % chord in ft.

% generalized A,B matrices 
statenamesFull = {'u','w','theta','q','h','beta','phi','p','r'}';

%flexI = 1:3;   % specify the flex states in generalized model (max 6)
Nflex = length(flexI);
for ii = 1:Nflex
    statenamesFull(9+Nflex+ii) = {['eta' num2str(flexI(ii)) 'dt']};
    statenamesFull(9+ii) = {['eta' num2str(flexI(ii))]};
end

[~,idx] = setdiff(statenamesFull,sysePID0.StateName);  % find state names not appearing in identified model
[idxA,idx2] = setdiff([1:9+2*Nflex],idx);  % indices of A to be filled

dummyA = zeros(length(statenamesFull));
dummyB = zeros(length(statenamesFull),size(B,2));

% dummyA = nan(length(statenamesFull));
% dummyB = nan(length(statenamesFull),size(B,2));

dummyA(idxA,idxA) = A;
dummyB(idxA,:) = B;

% transformation to account for beta
T = eye(length(statenamesFull));
T(6,6) = V;
dummyA = T*dummyA/T;
dummyB = T*dummyB;

%m = 13.76/2.21; % kg
m=massProp.m; 
Iyy=massProp.Iyy;%*14.59/3.28/3.28; % Iyy kg-m^2 Geri FEM 5.1
Ixx=massProp.Ixx;%*14.59/3.28/3.28;
Izz=massProp.Izz;%*14.59/3.28/3.28;  % From VT FEM 5.1 Geri
Ixz = 0;
omega = massProp.omega;
zeta = massProp.zeta;

% lift derivatives
coeffsout.CZ_alpha = m*dummyA(2,2)*V/q/S;
if dummyA(2,4) == 0
    coeffsout.CZ_q = 0;
else
    coeffsout.CZ_q = m*(dummyA(2,4)-U)*2*V/q/S/cbar;
end

% pitching moment derivatives
coeffsout.Cm_alpha = Iyy*dummyA(4,2)*V/q/S/cbar;
coeffsout.Cm_q = Iyy*dummyA(4,4)*2*V/q/S/(cbar^2);

% side force coeff
coeffsout.CY_beta = m*dummyA(6,6)*V/q/S;
coeffsout.CY_p = m*dummyA(6,8)*2*V/q/S/b;
if dummyA(6,9) == 0
    coeffsout.CY_r = 0;
else
    coeffsout.CY_r = m*(dummyA(6,9)+V)*2*V/q/S/b;
end

% drag force coeff
if dummyA(1,2) == 0
    coeffsout.CX_alpha = 0;
else
    CLtrim = m*32.17/q/S;
    coeffsout.CX_alpha = m*dummyA(1,2)*V/q/S - CLtrim;
end
if dummyA(1,4) == 0
    coeffsout.CX_q = 0;
else
    coeffsout.CX_q = m*(dummyA(1,4)+W)*2*V/q/S/cbar;
end


% roll coefficients
coeffsout.Cl_beta = Ixx*dummyA(8,6)*V/q/S/b;
coeffsout.Cl_p = Ixx*dummyA(8,8)*2*V/q/S/(b^2);  % correct
coeffsout.Cl_r = Ixx*dummyA(8,9)*2*V/q/S/(b^2);

% yaw coefficients
coeffsout.Cn_beta = Izz*dummyA(9,6)*V/q/S/b;
coeffsout.Cn_p = Izz*dummyA(9,8)*2*V/q/S/(b^2);
coeffsout.Cn_r = Izz*dummyA(9,9)*2*V/q/S/(b^2);% correct

% aeroelastic coefficients (match Dave's form)
% rigid forces and moments due to flex states
coeffsout.CZ_eta_i = m*dummyA(2,10:9+Nflex)/q/S;
coeffsout.CZ_etadt_i = m*dummyA(2,9+Nflex+1:end)*V/q/S;

coeffsout.CY_eta_i = m*dummyA(6,10:9+Nflex)/q/S;
coeffsout.CY_etadt_i = m*dummyA(6,9+Nflex+1:end)*V/q/S;

coeffsout.Cm_eta_i = Iyy*dummyA(4,10:9+Nflex)/q/S/cbar;
coeffsout.Cm_etadt_i = Iyy*dummyA(4,9+Nflex+1:end)*V/q/S/cbar;

coeffsout.Cl_eta_i = Ixx*dummyA(8,10:9+Nflex)/q/S/b;
coeffsout.Cl_etadt_i = Ixx*dummyA(8,9+Nflex+1:end)*V/q/S/b;

coeffsout.Cn_eta_i = Izz*dummyA(9,10:9+Nflex)/q/S/b;
coeffsout.Cn_etadt_i = Izz*dummyA(9,9+Nflex+1:end)*V/q/S/b;

% elastic forces due to rigid states
mgen = 1/12;  % match VT mass units
coeffsout.CQ_alpha = mgen*dummyA(9+Nflex+1:end,2)*V/q/S/cbar;
coeffsout.CQ_q = mgen*dummyA(9+Nflex+1:end,4)*V/q/S/cbar;
coeffsout.CQ_beta = mgen*dummyA(9+Nflex+1:end,6)*V/q/S/b;
coeffsout.CQ_p = mgen*dummyA(9+Nflex+1:end,8)*V/q/S/(b);  % correct
coeffsout.CQ_r = mgen*dummyA(9+Nflex+1:end,9)*V/q/S/(b);

% elastic forces due to flex states
omegasq = diag(omega(flexI).*omega(flexI));   % does not account for missing flex states
zetaomega = diag(2*omega(flexI).*zeta(flexI));
coeffsout.CQ_eta_i = mgen*(dummyA(9+Nflex+1:end,10:9+Nflex)+omegasq)/q/S/cbar;
coeffsout.CQ_etadot_i = mgen*(dummyA(9+Nflex+1:end,9+Nflex+1:end)+zetaomega)*V/q/S/cbar;

% control derivatives
inputnamesLong = {'Sym BF'
    'Sym IB'
    'Sym Mid'
    'Sym OB'
    'Thrust lb'};
inputnamesLat = { 'AS BF'
    'AS IB'
    'AS Mid'
    'AS Ob'};
inputnamesGust = { 'wGust'};
% [~,idxU] = setdiff(inputnamesFull,sysePID0.InputName);  % find names not appearing in identified model
% [idxB,idx3] = setdiff([1:10],idxU);  % indices of A to be filled

% separate longitudinal and lateral inputs, gust input 
[~,idxBLong] = intersect(sysePID0.InputName,inputnamesLong);
[~,idxBLat] = intersect(sysePID0.InputName,inputnamesLat);
[~,idxBgust] = intersect(sysePID0.InputName,inputnamesGust);


 
coeffsout.CZ_Li = m*dummyB(2,idxBLong)/q/S;
coeffsout.CX_Li = m*dummyB(1,idxBLong)/q/S;
coeffsout.Cm_Li = Iyy*dummyB(4,idxBLong)/q/S/cbar;
coeffsout.CZ_gust = m*dummyB(2,idxBgust)/q/S;
coeffsout.CX_gust = m*dummyB(1,idxBgust)/q/S;
coeffsout.Cm_gust = Iyy*dummyB(4,idxBgust)/q/S/cbar;


coeffsout.CY_Li = m*dummyB(6,idxBLat)/q/S;
coeffsout.Cl_Li = Ixx*dummyB(8,idxBLat)/q/S/b;
coeffsout.Cn_Li = Izz*dummyB(9,idxBLat)/q/S/b;
coeffsout.CY_gust = m*dummyB(6,idxBgust)/q/S;
coeffsout.Cl_gust = Ixx*dummyB(8,idxBgust)/q/S/b;
coeffsout.Cn_gust = Izz*dummyB(9,idxBgust)/q/S/b;


coeffsout.CQ_Li_long = mgen*dummyB(9+Nflex+1:end,idxBLong)/q/S/cbar;
coeffsout.CQ_Li_lat = mgen*dummyB(9+Nflex+1:end,idxBLat)/q/S/b;
coeffsout.CQ_Li_gust = mgen*dummyB(9+Nflex+1:end,idxBgust)/q/S/cbar;
