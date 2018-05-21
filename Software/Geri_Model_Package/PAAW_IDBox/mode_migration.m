function [lmbs,wns,zets,ord] = mode_migration(Afunc,alphas,opts)

%determine mode migration for a generic function of one parameter that outputs a matrix. eignevalues
%are order for smooth plots.
%[lmbs,wns,zets,ord] = mode_migration(Afunc,alphas)
%INPUTS:
%Afunc = function handle. function must take a scalar input and return a
%square matrix. can also output a ss model, in which case only the A
%matrix is used.
%alphas = vector of scalar parameters.
%opts = options structure (optional)
%OUTPUTS:
%lmbs = matrix of eigenvalues computed at each alphas value. each vector corresponds to a single
%       alphas value.
%wns,zets = frequencies and damping ratios corresponding to lmbs
%ord = matrix where vectors are the order of the original eigenvalues produced.

if nargin < 3
    opts.mac = 'traditional';
    opts.realimag = 1;
end

if ~isfield(opts,'mac')
    opts.mac = 'traditional';
end

if ~isfield(opts,'realimag')
    opts.realimag = 1;
end

%define mac computation
switch opts.mac
    case {'traditional','trad'} %the traditional formula. works for real, or mostly-real modes
        macfunc = @(v1,v2,lamb1,lamb2)abs(v1'*v2)^2/((v1'*v1)*(v2'*v2));
    case 'macx' %from Vacher, P., et. al. "Extensions of the MAC criterion to complex
                %modes," preceedings of ISMA 2010
                %works better for complex modes
        macfunc = @(v1,v2,lamb1,lamb2)(abs(v1'*v2) + abs(v1.'*v2))^2/((v1'*v1 + abs(v1.'*v1))*(v2'*v2 + abs(v2.'*v2)));
    case 'macxp' %same as 'macx' but with pole weighting. see same ref. NOT YET VALIDATED!!
        macfunc = @(v1,v2,lamb1,lamb2)(abs(v1'*v2)/abs(conj(lamb1) + lamb2) + abs(v1.'*v2)/abs(lamb1 + lamb2))^2/...
            ((v1'*v1/(2*abs(real(lamb1))) + abs(v1.'*v1)/(2*abs(lamb1)))*(v2'*v2/(2*abs(real(lamb2))) + abs(v2.'*v2)/(2*abs(lamb2))));
    otherwise
        error('opts.mac improperly set!!');
end

for ind = 1:length(alphas)
   Amod = Afunc(alphas(ind));
   if isa(Amod,'ss')
       Amod = Amod.a;
   end
   
   [V,D] = eig(Amod);  %eigenvalues
   evals = diag(D);
   
   if opts.realimag
       %eliminate modes with negative imag part
       ixr = find(imag(evals)>=0);
       evals = evals(ixr);
       V = V(:,ixr);
   end
   
   if ind == 1
       %sort by freq magnitude
       [~,ix] = sort(abs(evals));
       ord(:,ind) = ix; %[1:length(evals)]';
   else
       %sort using MAC by comparing to previous
       for ind2 = 1:size(Vold,2)
           for ind3 = 1:size(V,2)
            v1 = V(:,ind3);
            v2 = Vold(:,ind2);
            lamb1 = evals(ind3);
            lamb2 = evalsold(ind2);
            MAC(ind3) = macfunc(v1,v2,lamb1,lamb2); %abs(v1'*v2)^2/((v1'*v1)*(v2'*v2));
           end
%            ii = find(MAC == max(MAC));
           [MAC,ix] = sort(MAC,'descend');
           ii = ix(1);
%            if ind2>1 && ~isempty(find(ord(:,ind)) == ii)
%                ii = ix(2);
%            end
           ord(ind2,ind) = ii;
       end
       ix = ord(:,ind);
   end
   
   V = V(:,ix);
   lmbs(:,ind) = evals(ix);
   [wns(:,ind),zets(:,ind)] = damp(evals(ix));
   
   Vold = V;
   evalsold = evals(ix);
end




