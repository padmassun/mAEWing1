function params = build_paramvec(coeffdata,coefffree)

%build a parameter vector given coeffdata and coefffree

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
                    params(pind) = dat(rw,cl);  %assign
                end
            end
        end
    end
end

params = params(:);