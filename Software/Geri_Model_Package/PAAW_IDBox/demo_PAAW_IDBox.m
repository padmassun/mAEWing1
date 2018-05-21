%demo to excercise functions in PAAW_IDBox

%% load model and extract coefficients

clear all

%load model files
load Geri_data\Geri_Database.mat   %database for NDOF function
load Geri_data\GeriaeroPropNOM.mat %aero properties
load Geri_data\GericoeffsNOM.mat   %nominal coefficients
load Geri_data\GerimassProp.mat    %msas properties

%set velocity
ft2m = 0.3048;
Vmps = 23; %velocity in m/s
aeroProp.V = Vmps/ft2m; %need to set the proper value in aeroProp structure

flexI = [1:6];  %which flex modes to retain
IputIx = [1:9]; %inputs represented with coeffdata

coeffdata.flexI = flexI;
coeffdata.IputIx = IputIx;

%build ss system
GeriFDsys = Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffdata);

%extract coefficients from model
coeffdata_out = getcoeffsFDmod(GeriFDsys,massProp,aeroProp,flexI); %coeffdata_out = coeffdata