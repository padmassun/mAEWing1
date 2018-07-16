function sys_full = NdofwActSens(okeep,ikeep,AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata,actdata,sensdata)

%generate state space system with selected I/O that includes actuator and sensor dynamics
%INPUTS:
%okeep = vector of output indices to retain
%ikeep = vector of input indices to retain
%AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata = same inputs to Ndof6_FDFLEX
%actdata = structure with actuator data as transfer functions (not state space)
%sensdata = structure with sensor data as transfer functions (not state space)
%
%OUTPUTS;
%sys_full = resulting state space system with inputs, outputs, and states labeled (including units). 
%           this system inludes actuator and sensor dynamics
%*this system has direct individual surface inputs, not sym and asym input pairs
%
%Brian Danowsky, Systems Technology, Inc. 2018

%1st get the bare airframe model (note that direct inputs are hard-coded)
sys_bare = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata,'DirectInp');

%define full actuator system ---------
G_surface_actuator = actdata.G_surface_actuator;
engine_lag = actdata.engine_lag;

ACT = blkdiag(G_surface_actuator,G_surface_actuator,G_surface_actuator,G_surface_actuator,...
    G_surface_actuator,G_surface_actuator,G_surface_actuator,G_surface_actuator,engine_lag,1);
ACT.inputname = sys_bare.inputname;
ACT.inputunit = sys_bare.inputunit;

%define full sensor system------------
G_sens_IMU = sensdata.G_sens_IMU;
G_sens_Accel = sensdata.G_sens_Accel;

SENS = blkdiag(G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,...
    G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_Accel,G_sens_Accel,G_sens_Accel,...
    G_sens_Accel,G_sens_Accel,G_sens_Accel,G_sens_Accel,G_sens_IMU,G_sens_IMU,G_sens_IMU,...
    G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_IMU,G_sens_Accel,...
    G_sens_Accel,G_sens_Accel);
SENS.outputname = sys_bare.outputname;
SENS.outputunit = sys_bare.outputunit;

%reduced actuator and sensor systems based on selected I/O----------
ACT2use = ss(ACT(ikeep,ikeep)); %if ACT is a tf, appropriate states will be truncated with the selected inputs
for ind = 1:size(ACT2use.a,1)
    ACT2use.statename{ind} = ['actuator(' num2str(ind) ')'];
end

SENS2use = ss(SENS(okeep,okeep));
for ind = 1:size(SENS2use.a,1)
    SENS2use.statename{ind} = ['sensor(' num2str(ind) ')'];
end

%build complete system----------
sys_full = SENS2use*sys_bare(okeep,ikeep)*ACT2use;


