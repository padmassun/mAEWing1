function [A,B,C,D] = odeGerifun_grey(params,Ts,opt_params)

%params to build model
AE6 = opt_params.AE6; %for Ndof6_FDFLEX
ModeShape = opt_params.ModeShape; %for Ndof6_FDFLEX
FEM = opt_params.FEM; %for Ndof6_FDFLEX
Vmps = opt_params.Vmps; %airspeed in m/s
aeroProp = opt_params.aeroProp; %aero properties
massProp = opt_params.massProp; %mass properties
coeffdataIC = opt_params.coeffdataIC; %IC default parameters. has flexI and IputIx fields imbedded.      

%model reduction parameters
flexI = opt_params.flexI;   %which flex modes to retain
elimrb = opt_params.elimrb; %which RB to eliminate

%inputs and outputs
outs2use = opt_params.outs2use;
ins2use = opt_params.ins2use;
measi = opt_params.measi;

%weighting parameters
Oweight = opt_params.Oweight;
Iweight = opt_params.Iweight;

SCA = opt_params.SCA;

%define which parameters are free
coefffree = opt_params.coefffree;

%get the ss model
coeffdata = rep_coeff_params(params,coefffree,coeffdataIC); %replace coefficients with values in params given which are free
sysfull = Ndof6_FDFLEX(AE6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata);

allmodes = 1:6;
keeps = zeros(1,6);
keeps(flexI) = 1;
elimmodes = allmodes(~keeps);

% elimstates = [1,3,5,7,9+elimmodes,15+elimmodes];
elimstates = [elimrb,9+elimmodes,15+elimmodes];
sysredfull = modred(sysfull,elimstates,'truncate');

sysfullID = sysredfull(outs2use,ins2use);

%scale outputs to match data if specified
sofnames = sysfullID.outputname;
sysfullID = SCA*sysfullID;
sysfullID.outputname = sofnames;

sysfullID = sysfullID(measi,:);

sysweighted = Oweight*sysfullID*Iweight; %full I/O weighted system

[A,B,C,D] = ssdata(sysweighted);



