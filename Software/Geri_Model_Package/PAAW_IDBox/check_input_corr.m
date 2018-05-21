function [rhoisv2,rho2s,wcohere,Su,X] = check_input_corr(u,dt,inames,stu)

%check correlation of inputs.
%INPUTS:
%u = input vectors as columns
%stu = FREDA settings. option. defaults are below if not supplied
%OUTPUTS:
%rhoisv2 = input singular value coherence (see WP-2702-1, sec. 2.9)
%rho2s, wcohere = outputs of sigcohere.m. see Utility in STITools
%Su, X = matrices used to get rhoisv2. (see WP-2702-1, sec. 2.9)

if nargin<3
    inames = [];
end

%check input linear independence (see section 2.9 of WP-2702-1) (do this beofre any pre-processing)-
for ind = 1:size(u,2)
    X(:,ind) = u(:,ind)./norm(u(:,ind)); %normalize inputs
end
Su = X'*X;

rhoisv2 = (1-min(svd(Su)))^2;
%rhoisv < 0.5 (very good)
%rhoisv < 0.9 (good)
%rhoisv < 0.99 (okay)

%check coherence of all inputs ---------------------------------------------------------------------
stu.dt = dt;
if (nargin < 4) || isempty(stu) %default settings
    stu.useSmoothing = 1;
    stu.psdTaper=0.0; % taper fraction (this is done already)
    stu.binRatio=1.1; % bin ratio
    stu.binSize=3; % average at least this many points
    stu.zeroFill=0; % seconds of zero fill
end

%time domain counterpart
% rhot = X(2:end-1,1).*X(2:end-1,2)./(X(2:end-1,1).^2.*X(2:end-1,2).^2);
%---------------------------------------------------------------------------------------------------

[rho2s,wcohere] = sigcohere(u,stu,inames);
title(['Coherence between all inputs (\rho_{isv}^2 = ' num2str(rhoisv2,'%3.2e') ')']);