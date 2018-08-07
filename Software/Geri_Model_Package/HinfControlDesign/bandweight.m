function [ bandweight ] = bandweight(int_wc, diff_wc )
%BANDWEIGHT 
% [ bandweight ] = bandweight(int_wc, diff_wc )
% weight with integral behavior up to int_wc, then unit gain up to diff_wc,
% then differentiating behavior
bandweight = 2*intweight(int_wc/2)*diffweight(diff_wc);

end

