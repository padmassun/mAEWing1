function [Tcont, Tctrb] = ssctr(A, B)
%multivariable controllability canonical form
% according to Kailath Scheme I (crate column search) (p426ff)

C = [];


% Kailath Scheme I (crate column search)
for jj = 1:size(B,2)
    for ii = 1:size(A,2)
        Ctmp = [C A^(ii-1)*B(:,jj)];
        if rank(Ctmp,0.01) > rank(C,0.01) % if rank gained
            C = Ctmp;                     % take vector to ctrb
            if rank(C,0.01) == size(A,2)
                break
            end
        end
    end
    if rank(C,0.01) == size(A,2)
        break
    end
end


%controllability canonical form
Tctrb = C^-1;
Tctrb*A/Tctrb;
Tctrb*B;

% controller-type form
c = Tctrb(size(A,2),:);

Tcont = [];

for ii = 1:size(A,2)
    Tcont = [Tcont ; c*A^(ii-1)];
end

Tcont*A/Tcont;
Tcont*B;
