function coeffdataout = rep_coeff_params(params,coefffree,coeffdata)

%replace coeffdata with values in vector params given coefffree

coeffdataout = coeffdata; %1st set to defaults
coeffields = fields(coeffdata);

%reset values
pind = 0; %parameter vector index
for ind = 1:length(coeffields)
    cfieldname = coeffields{ind};
    if ~strcmp(cfieldname,'flexI') && ~strcmp(cfieldname,'IputIx')
        dat = getfield(coeffdata,cfieldname);
        free = getfield(coefffree,cfieldname);
        for rw = 1:size(dat,1)
            for cl = 1:size(dat,2)
                if free(rw,cl)
                    pind = pind+1;
                    dat(rw,cl) = params(pind);  %assign
                end
            end
        end
        coeffdataout = setfield(coeffdataout,cfieldname,dat);
    end
end
