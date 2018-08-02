function [hRu,wu,rho2sm,rho2_isvs,rho2sm_bc,w_bc] = FRD_refine(tfdat,measi,wrange,rho2cut,inames,outnamesuse)

%refine FRD data: keep only a certain freq range and values above a coherence cuttoff

wmin = wrange(1);
wmax = wrange(2);

iws = find(tfdat.w>wmin,1,'first');
iwe = find(tfdat.w<wmax,1,'last');

rho2s = tfdat.rho2(measi,iws:iwe);
if min(size(rho2s)) == 1
    rho2sm = rho2s;
else
    rho2sm = min(rho2s);  %minimum rho2 across all signals
end

rho2_isvs = tfdat.rho2_isv(iws:iwe);

rho2sm_bc = rho2sm; w_bc = tfdat.w(iws:iwe); %save prior to cut values

hu = tfdat.h(measi,:,iws:iwe);
wu = tfdat.w(iws:iwe);

%eliminate cuttoff coherence
hu = hu(:,:,rho2sm>rho2cut);
wu = wu(rho2sm>rho2cut);
rho2sm = rho2sm(rho2sm>rho2cut);
rho2_isvs = rho2_isvs(rho2sm>rho2cut);
hRu = frd(hu,wu);
hRu.inputname = inames;
hRu.outputname = outnamesuse;