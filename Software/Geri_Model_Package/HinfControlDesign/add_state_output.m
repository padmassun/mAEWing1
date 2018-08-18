function G_aug = add_state_output(G,states)
%--------------------------------------------------------------------------
% Syntax: G_aug = add_state_output(G, [states])
%--------------------------------------------------------------------------
% Adds the states as outputs to the system such that lft command can be
% used on augmented state feedback controllers.
%--------------------------------------------------------------------------

% ver 2014-04-16
[ny, nu] = size(G);
nx = size(G.a,1);

if nargin == 1
    states = 1:nx;
end

Ieye = eye(nx);

if isa(G,'pss')
    G_aug = pss(G.a,G.b,[G.c;Ieye(states,:)],[G.d;zeros(numel(states),nu)]);
    G_aug.InputName=G.InputName;
    G_aug.StateName=G.StateName;
    G_aug.OutputName=[G.OutputName;G.StateName(states)];
elseif isa(G,'ss')
    tmp = size(G);
    pdim = tmp(3:end);
    G = G(:,:,:);
    if size(G,3) == 1
        pdim=[1 1];
    end
    ngrid = prod(pdim);
    G_tmp = ss(zeros(ny+numel(states),nu,ngrid));
    for kk = 1:ngrid
        G_tmp(:,:,kk) = ss(G(:,:,kk).a, G(:,:,kk).b, [G(:,:,kk).c; Ieye(states,:)], [G.d(:,:,kk); zeros(numel(states),nu)]);
    end
    G_aug = reshape(G_tmp,pdim);
    set(G_aug,'InputName',G.InputName,'StateName',G.StateName,'OutputName',[G.OutputName;G.StateName(states)])
end