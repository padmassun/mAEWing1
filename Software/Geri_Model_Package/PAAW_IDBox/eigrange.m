function [lm,Vm] = eigrange(A,wrng)

%find eigenvalues and eigenvectors of a matrix in a certain frequency range
% [lm,Vm] = eigrang(A,wrng)
%INPUTS
%A = matrix, or SS model in which case it uses only the A matrix
%wrng = 2 element vector defining frequency range [wmin,wmax], in rad/s
%
%OUTPUTS
%lm = eigenvalues in the freq range
%Vm = eigenvectors in the freq range

if isa(A,'ss')
    A = A.a;
end

[V,D] = eig(A);

lmbs = diag(D);
[lmbs,ix] = sort(lmbs);
V = V(:,ix);

imax = max(find(abs(lmbs) <= wrng(2)));
imin = min(find(abs(lmbs) >= wrng(1)));

lm = lmbs(imin:imax);
Vm = V(:,imin:imax);