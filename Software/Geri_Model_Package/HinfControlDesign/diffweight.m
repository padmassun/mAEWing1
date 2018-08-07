function [ diffweight ] = diffweight(wa, decades)
%DIFFWEIGHT diffweight(wa, dec)
% weight with unit gain up to frequency wa, then differentiating behavior
% for dec decades (default 2)
if nargin <2
    decades = 2;
end

diffweight = ss(tf([1/wa 1],[1/(wa*10^decades) 1])); 

end

