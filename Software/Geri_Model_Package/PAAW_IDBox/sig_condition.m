function [uf,yf,lpfilt] = sig_condition(t,u,y,conflags)

%pre-condition input and output signals
%[uf,yf] = sig_condition(t,u,y,conflags)
%Inputs:
%t,u,y = time vector, input signasl as columns, output signals as columns
%conflags = structure of flags for conditioning
%   .dofilt = (1 = low pass filter filter data)
%   .filtfreqHz = the low pass filter frequency in Hz
%   .bord = filter order (Butterworth)
%   .dodetrend = (1 = detrend both u and y)
%   .docosine taper = (1 = cosine taper both u and y)
%   .tfr = cosine taper fraction is .docosinetaper = 1.
%   .doact = filter u through actuator model
%   .ACTsys = the actuator model if .doact = 1
%   .Tdelsteps = assumed number of time delay steps (integer) added to u. only done if .doact = 1
%OUTPUTS:
%uf, yf = conditioned u and y

%get flags
if isfield(conflags,'dofilt') && ~isempty(conflags.dofilt) 
    dofilt = conflags.dofilt;
else
    dofilt = 0;
end

if isfield(conflags,'filtfreqHz') && ~isempty(conflags.filtfreqHz) 
    filtfreqHz = conflags.filtfreqHz;
else
    filtfreqHz = 1/(t(2) - t(1)); %default is 2X Nyquist freq
end

if isfield(conflags,'bord') && ~isempty(conflags.bord) 
    bord = conflags.bord;
else
    bord = 5;
end

if isfield(conflags,'dodetrend') && ~isempty(conflags.dodetrend) 
    dodetrend = conflags.dodetrend;
else
    dodetrend = 0;
end

if isfield(conflags,'docosinetaper') && ~isempty(conflags.docosinetaper) 
    docosinetaper = conflags.docosinetaper;
else
    docosinetaper = 0;
end

if isfield(conflags,'tfr') && ~isempty(conflags.tfr) 
    tfr = conflags.tfr;
else
    tfr = 0.05;
end

if isfield(conflags,'doact') && ~isempty(conflags.doact) 
    doact = conflags.doact;
else
    doact = 0;
end

if isfield(conflags,'ACTsys') && ~isempty(conflags.ACTsys) 
    ACTsys = conflags.ACTsys;
else
    ACTsys = tf(1);
end

if isfield(conflags,'Tdelsteps') && ~isempty(conflags.Tdelsteps) 
    Tdelsteps = conflags.Tdelsteps;
else
    Tdelsteps = 0;
end

if isfield(conflags,'dosens') && ~isempty(conflags.dosens) 
    dosens = conflags.dosens;
else
    dosens = 0;
end

if isfield(conflags,'SENSsys') && ~isempty(conflags.SENSsys) 
    SENSsys = conflags.SENSsys;
else
    SENSsys = tf(1);
end

%condition the data

if dofilt
    %low pass filter data
    wcut = filtfreqHz*2*pi(); %cuttoff frequency (rad/s)
    if dofilt == 2  %a trick to get a high pass filter
        [nb,db] = butter(bord,wcut,'high','s');
    else
        [nb,db] = butter(bord,wcut,'s');  %this will be bandpass if wcut is 2 elements
    end
    lpfilt = tf(nb,db);

    tfvec = lpfilt;
    for ind = 2:size(u,2)
        tfvec = [tfvec,lpfilt];
    end
    lpfiltu = tfdiag(tfvec);

    tfvec = lpfilt;
    for ind = 2:size(y,2)
        tfvec = [tfvec,lpfilt];
    end
    lpfilty = tfdiag(tfvec);

    uf = lsim(lpfiltu,u,t);
    yf = lsim(lpfilty,y,t);

    uf = lsim(lpfiltu,flipud(uf),t);
    yf = lsim(lpfilty,flipud(yf),t);

    uf = flipud(uf);
    yf = flipud(yf);
else
    uf = u;
    yf = y;
end

if dodetrend
    %detrend
    uf = detrend(uf);
    yf = detrend(yf);
end

if docosinetaper
    %cosine taper
    uf = cosineTaper(uf,tfr);
    yf = cosineTaper(yf,tfr);
end

if doact
    %send u through actuator
    uf = lsim(ACTsys,uf,t);
    
    uf = [repmat(uf(1,:),Tdelsteps,1);uf(1:end-Tdelsteps,:)];  %add the time delay
end

if dosens
    %send y backwards through sensor model
    dt = t(2) - t(1);
    for ind = 1:size(yf,2)
        [b,a] = tfdata(c2d(SENSsys(ind,ind),dt));
        yf(:,ind) = filtrev(a{1},b{1},yf(:,ind));
    end
end



