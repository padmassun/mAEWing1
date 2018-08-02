function varypzmap(sys,map)
% varypzmap(sys,[map]) plots the varying pole-zero-map of an array of models.
%
% sys:  ss-array of system under consideration (i.e., if pss objects are used, use pss.Data instead)
% map:  optional color map


if nargin==1
map = 'CubicYF';
end

    sys = sys(:,:,:);
    ngrid = size(sys,3);
    if exist('pmkmp','file') && ~strcmp(map,'gray')
        col = flipud(pmkmp(ngrid,map));
    else
        col = flipud(gray(ngrid+1));
        col = col(2:end,:);
    end
    figure
    hold on
    for kk=1:ngrid
        [p, z] = pzmap(sys(:,:,kk));
        plot(real(p),imag(p),'x','Color',col(kk,:))
        plot(real(z),imag(z),'o','Color',col(kk,:))
    end

colormap(col)
colorbar