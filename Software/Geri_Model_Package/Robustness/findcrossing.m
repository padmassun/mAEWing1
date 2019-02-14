function [xcross] = findcrossing(xdata,ydata,ythresh)

%find when data crosses a threshold (interpolates)
%assumes ydata is above threshold at low xdata values and crosses at a higher xdata value (negative
%slope)

ix = find(ydata<ythresh,1,'first');
if ix == 1
    xcross = xdata(1);
else
    xcross = interp1(ydata(ix-1:ix),xdata(ix-1:ix),ythresh);
end