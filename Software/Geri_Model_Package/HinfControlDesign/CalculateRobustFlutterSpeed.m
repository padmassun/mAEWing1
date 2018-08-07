function [RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P, OCM_P, ICM_G, OCM_G, Vinf)
%CALCULATEROBUSTFLUTTERSPEED 
% [RFS, AFS, vis] = CalculateRobustFlutterSpeed(ICM_P, OCM_P, ICM_G, OCM_G, Vinf)
%
% ICM_P : matrix of minimum input classical phase margin over airspeed
% ICM_G : matrix of minimum input classical gain margin over airspeed
% OCM_P : matrix of minimum output classical phase margin over airspeed
% OCM_G : matrix of minimum output classical gain margin over airspeed
% (Are all caluclated by DISPLAYLOOPMARGIN)
% Vinf  : vector of airspeed grid
%
% RFS : Robust Flutter Speed
% AFS : Absolut Flutter Speed
% vis : structure containing x and y vectors for plotting box in margin
% over airspeed plot

[~,Vidx] = find(ICM_P<=45);
RFS_ICM_P = Vinf(min(Vidx)-1);
[~,Vidx] = find(ICM_P==0);
AFS_ICM_P = Vinf(min(Vidx));

[~,Vidx] = find(OCM_P<=45);
RFS_OCM_P = Vinf(min(Vidx)-1);
[~,Vidx] = find(OCM_P==0);
AFS_OCM_P = Vinf(min(Vidx));

[~,Vidx] = find(abs(db(ICM_G))<=6);
RFS_ICM_G = Vinf(min(Vidx)-1);
[~,Vidx] = find(abs(ICM_G)==0);
AFS_ICM_G = Vinf(min(Vidx));

[~,Vidx] = find(abs(db(OCM_G))<=6);
RFS_OCM_G = Vinf(min(Vidx)-1);
[~,Vidx] = find(abs(OCM_G)==0);
AFS_OCM_G = Vinf(min(Vidx));

RFS = min([RFS_ICM_P, RFS_ICM_G, RFS_OCM_P, RFS_OCM_G]);
AFS = min([AFS_ICM_P, AFS_ICM_G, AFS_OCM_P, AFS_OCM_G]);

% vis.RFS_P_x = [Vinf(1), RFS, RFS, Vinf(1), Vinf(1)];
% vis.RFS_P_y = [0, 0, 45, 45, 0];
% vis.RFS_G_x = [Vinf(1), RFS, RFS, Vinf(1), Vinf(1)];
% vis.RFS_G_y = [0, 0, 6, 6, 0];
vis.RFS_P_x = [Vinf(1), RFS, RFS, Vinf(1)];
vis.RFS_P_y = [0, 0, 45, 45];
vis.RFS_G_x = [Vinf(1), RFS, RFS, Vinf(1)];
vis.RFS_G_y = [0, 0, 6, 6];

end

