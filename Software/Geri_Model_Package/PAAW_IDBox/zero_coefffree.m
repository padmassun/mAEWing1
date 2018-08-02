function coefffree = zero_coefffree(coeffdata)

%build coefffree of zeros using coeffdata to get the required fields
coefffree = coeffdata; %has same fields
coeffields = fields(coeffdata);
%loop through
for ind = 1:length(coeffields)
    cfieldname = coeffields{ind};
    dat = getfield(coeffdata,cfieldname);
    for rw = 1:size(dat,1)
        for cl = 1:size(dat,2)
            dat(rw,cl) = 0;  %make them all zero to initialize
        end
    end
    coefffree = setfield(coefffree,cfieldname,dat);
end

%remove unneeded fields
if isfield(coefffree,'flexI')
    coefffree = rmfield(coefffree,'flexI');
end

if isfield(coefffree,'IputIx')
    coefffree = rmfield(coefffree,'IputIx');
end