function Gweighted = genplant(syn, We, Wu, W1, W2, Wy, Wp)
% Gweighted = genplant(syn, We, Wu, W1, W2, [Wy], [Wp])
% Produces a generalized plant of the following structure:
%  W1 needs to include all inputs and W2 needs to incluide all measurable
%  outputs. The plant SYN must be grouped as [G;P], that is the first
%  outputs are the measurable ones. Additional performance outputs follow.
%
%     |    W1      W2
%--------------------
% We  |    SG      S
% Wu  |    Ti      KS
% - - - - - - - - - -
% Wy  |    SG      T
% Wp  |   PSi     PKS
% 
% WARNING: inputs reordered compared to version 1/22/2016!
%
% 2016-9-6 JT

[nout, nu] = size(syn);
if nargin <7
    Wp = [];
end
if nargin <6
    Wy = [];
end

np = size(Wp,2);
ny = nout-np;

if isempty(W1)
    W1 = zeros(nu,0);
end
if isempty(W2)
    W2 = zeros(ny,0);
end
if isempty(We)
    We = zeros(0,ny);
end
if isempty(Wy)
    Wy = zeros(0,ny);
end
if isempty(Wp)
    Wp = zeros(0,np);
end

if nu~=size(W1,1) 
    error('input dimensions do not agree')
elseif ny~=size(W2,2)
    error('output dimensions do not agree')
elseif any(nu~=size(Wu)) 
    error('Wu dimensions do not agree')
end

systemnames = 'syn W1 W2 We Wy Wu Wp';
inputvar = ['[d{' int2str(size(W1,2)) '}; n{' int2str(ny) '}; u{' int2str(nu) '}]'];
outputvar = ['[We; Wy; Wu; Wp; W2-syn(1:' int2str(ny) ')]'];
input_to_syn = '[ u+W1 ]';
input_to_W1 = '[d]';
input_to_W2 = '[n]';
input_to_Wu = '[u]';
input_to_Wy = ['[syn(1:' int2str(ny) ')]'];
input_to_We = ['[W2-syn(1:' int2str(ny) ')]'];
input_to_Wp = ['[syn(' int2str(1+ny) ':' int2str(nout) ')]'];
Gweighted = sysic;

end

