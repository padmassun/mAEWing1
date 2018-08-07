function [sysmod, T, Tinv] = modalform(sys)
% perform "canon" transformation to modal form and sort modes with
% ascending frequency

[ny, nu] = size(sys);
nx = size(sys.a,1);
[sysmod, Tmod] = canon(sys);
EYE = eye(nx);

% extract block count and frequencies. blockcount tells whether a state
% belongs to a real(1) or complex (2) block.
% freq is the frequency associated with the state
% nblocks is the number of blocks in each grid point
[blockcount, frq, dmp, rblocks, cblocks] = get_block_count(sysmod,0);


    % sort all blocks by acending block size to avoid problems with block changes information used during sorting is T, freq and blockcount.
    [blockcount, ind_block] = sort(blockcount);
    
        Tmod = Tmod(ind_block,:);
        frq = frq(ind_block);
        dmp = dmp(ind_block);
        sysmod(:,:) = ss2ss(sysmod(:,:),EYE(ind_block,:));
    

    % sort all blocks by acending frequency for convenience.
    ind_freq = zeros(nx,1);
    
        [frq(1:rblocks), ind_freq(1:rblocks)] = sort(frq(1:rblocks));
        [freq_tmp, ind_freq_tmp] = sort([zeros(rblocks,1); frq(1+rblocks:end)]);
        [frq(1+rblocks:end), ind_freq(1+rblocks:end)] = deal(freq_tmp(1+rblocks:end), ind_freq_tmp(1+rblocks:end));
        
        
        Tmod = Tmod(ind_freq,:);
        blockcount = blockcount(ind_freq); 
        dmp = dmp(ind_freq);  
        sysmod = ss2ss(sysmod,EYE(ind_freq,:));


if nargout==0
    return  %just display the number of blocks
end

% get right eigenvetors also. LEV are rows of T, REV are columns of Tinv
    Tinv = Tmod^-1;





% #########################################################################


% AUX FUNCTIONS
function [blocksize, nblocks] = get_block_size(sysmodData)
% detect blocks in a matrix
% blocksize{kk}: [ 1 1 2 2 2 1]   means 2 real, than 3 complex, than 1
%                                 real block
% nblocks(kk): number of blocks on grid point kk
nx = size(sysmodData.a,1);
ngrid = size(sysmodData,3);


for kk=1:ngrid
    if ~isbanded(sysmodData(:,:,kk).a ,1,1)
%         keyboard
        error(['Real Canonical Form not banded @' int2str(kk)])
    end
    
    for ii=1:nx-1
        break_index(ii) = abs(sysmodData.a(ii+1,ii,kk))<1e-8;
    end
    break_points = 1:nx;
    break_points = break_points(break_index);
    blocksize{kk} = diff([0, break_points, nx]);
    
    if any(blocksize{kk}>2) || any(blocksize{kk}==0);
        error('block determination failed');
    end
    nblocks(kk) = numel(blocksize{kk});
end




function [blockcount, freq, dmp, rblocks, cblocks] = get_block_count(sysmodData, verbose)
%[blockcount, freq, nblocks] = get_block_count(sysmodData)
%
% blockcount(ii,kk): size of block corresponding to entry ii on grid index kk
% freq(ii,kk): frequency corresponding to entry ii on grid index kk
% nblocks(kk): number of blocks on grid point kk
% rblocks(kk): number of real blocks on grid point kk
% cblocks(kk): number of complex blocks on grid point kk
nx = size(sysmodData.a,1);
ngrid = size(sysmodData,3);
[freq, dmp, blockcount] = deal(zeros(nx,ngrid));

[blocksize, nblocks] = get_block_size(sysmodData);


for kk=1:ngrid
    tt=1;
    for ii = 1:nblocks(kk)
        block = sysmodData.a(tt:tt+blocksize{kk}(ii)-1,tt:tt+blocksize{kk}(ii)-1,kk);
        [freq(tt:tt+blocksize{kk}(ii)-1,kk), dmp(tt:tt+blocksize{kk}(ii)-1,kk)] = damp(block);
        blockcount(tt:tt+blocksize{kk}(ii)-1,kk) = blocksize{kk}(ii);
        tt = tt+blocksize{kk}(ii);
    end
    
    rblocks(kk) = numel(find(blockcount(:,kk)==1));
    cblocks(kk) = numel(find(blockcount(:,kk)==2))/2;
    if verbose
        fprintf(' @ grid %2d:',kk)
        fprintf(' Blocks: %2d',nblocks(kk))
        fprintf('   C-Blocks: %2d',cblocks(kk))
        fprintf('   R-Blocks: %2d\n',rblocks(kk))
    end
    
end
%         vals(complblks_kk)  = vals_tmp(complblks_kk);
%         [vals_tmp, ind_tmp] = max(obj_real,[],2);
%         ind(realblks_kk)    = ind_tmp(realblks_kk);
%         vals(realblks_kk)   = vals_tmp(realblks_kk);
