function [tabheader,tabdat,coeffields,tabcolors,nfreecoeffs] = make_coefftable_comp(coeffsIC, coeffsPID, coefffree)

%build the header and data files used to build a Word table comparing the coefficients between 2
%models.
%INPUTS:
%coeffsIC = initial conditoin coefficients
%coeffsPID = coefficients resulting from PID
%coefffree = optional showing which coeffs are free for highlighting purposes.
%OUTPUTS:
%tabheader = cell cpontaining the table headers
%tabdat = cell containing the table data
%coeffields = the fields in coeffsIC

if nargin<3;coefffree=[];end

tabheader = {'Coefficient','Initial Value','Final Value','Percent Difference from Initial'};

coeffields = fields(coeffsIC);
nfreecoeffs = 0;
row = 1;
for ind = 1:length(coeffields)
    cfieldname = coeffields{ind};
    if ~strcmp(cfieldname,'flexI') && ~strcmp(cfieldname,'IputIx')
        ICdat = getfield(coeffsIC,cfieldname);
        PIDdat = getfield(coeffsPID,cfieldname);
    %     try
            percdiff = 100*(PIDdat - ICdat)./abs(ICdat);
    %     catch
    %         hey = 1;  %this is only for error catching if there is a bug
    %     end
        if isscalar(ICdat)
            tabdat(row,:) = {cfieldname,ICdat,PIDdat,[num2str(percdiff,'%5.1f') ' %']};
            if ~isempty(coefffree) && getfield(coefffree,cfieldname)
                nfreecoeffs = nfreecoeffs+1; %count number of free coefficients
                tabcolors(row,:) = {'lightyellow','lightyellow','lightyellow','lightyellow'};
            else
                tabcolors(row,:) = {'','','',''};
            end
            row = row+1;
        else
            if ~isempty(coefffree)
                freei = getfield(coefffree,cfieldname);
            else
                freei = zeros(size(ICdat));
            end
            for cl = 1:size(ICdat,2)
                for rw = 1:size(ICdat,1)
                    tabdat(row,:) = {[cfieldname, '(' num2str(rw) ,',' num2str(cl) ')'],ICdat(rw,cl),PIDdat(rw,cl),[num2str(percdiff(rw,cl),'%5.1f') ' %']};
                    if freei(rw,cl)
                        nfreecoeffs = nfreecoeffs+1; %count number of free coefficients
                        tabcolors(row,:) = {'lightyellow','lightyellow','lightyellow','lightyellow'};
                    else
                        tabcolors(row,:) = {'','','',''};
                    end
                    row = row+1;
                end
            end
        end
    end
end