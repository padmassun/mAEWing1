function [ intweight ] = intweight(wb, lf)
%INTWEIGHT 
% [ intweight ] = intweight(wb, lf)
% weight with low frequency gain lf (default 1e5) and integral behavior
% up to frequency wb, then gain 0.5
if nargin <2
    lf = 1e5;
end

intweight = makeweight(lf,wb,0.5); 

end

