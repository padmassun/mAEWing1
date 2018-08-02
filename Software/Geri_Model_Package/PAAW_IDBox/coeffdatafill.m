function coeffsPIDout = coeffdatafill(coeffsPIDin,coeffdata,elimrb)

%take a partial coeffdata from a reduced system with less than full inputs and fill values from a
%full set of coeffs.

coeffsPIDout = coeffdata; %1st fill it all in with the defaults
% coeffsPIDout.CQ_Li = [coeffsPIDout.CQ_Li_long,coeffsPIDout.CQ_Li_lat]; %make a complete CQ_Li, no _long or _lat

coeffields = fields(coeffsPIDin); %get fields

%get long and lat indexes
IputIx = coeffsPIDin.IputIx;
IputIxlong = IputIx(IputIx<6);
IputIxlat = IputIx(IputIx>5) - 5;

%get retained flex modes
flexI = coeffsPIDin.flexI;

for ind = 1:length(coeffields)
    cfieldname = coeffields{ind};
    if ~strcmp(cfieldname,'flexI') && ~strcmp(cfieldname,'IputIx')
        if isempty(strfind(cfieldname,'_gust'))
            dat = getfield(coeffsPIDin,cfieldname);
            if ~isempty(strfind(cfieldname,'X_'))
                if ~any(elimrb == 1)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlong) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end
            elseif ~isempty(strfind(cfieldname,'Z_'))            
                if ~any(elimrb == 2)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlong) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end            
            elseif ~isempty(strfind(cfieldname,'m_'))            
                if ~any(elimrb == 4)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlong) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end            
            elseif ~isempty(strfind(cfieldname,'Y_'))            
                if ~any(elimrb == 6)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlat) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end            
            elseif ~isempty(strfind(cfieldname,'l_'))            
                if ~any(elimrb == 8)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlat) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end            
            elseif ~isempty(strfind(cfieldname,'n_'))            
                if ~any(elimrb == 9)
                    if ~isempty(strfind(cfieldname,'_Li'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(IputIxlat) = dat;
                    elseif ~isempty(strfind(cfieldname,'_eta'))
                        dati = getfield(coeffdata,cfieldname);
                        dati(flexI) = dat;
                    else
                        dati = dat;
                    end
                    coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
                end            
            elseif ~isempty(strfind(cfieldname,'Q_'))
                dati = getfield(coeffdata,cfieldname);
                if ~isempty(strfind(cfieldname,'_Li_long'))
                    dati(flexI,IputIxlong) = dat;
                elseif ~isempty(strfind(cfieldname,'_Li_lat'))
                    dati(flexI,IputIxlat) = dat;
                elseif ~isempty(strfind(cfieldname,'_eta'))
                    dati(flexI,flexI) = dat;
                else
                    dati(flexI) = dat;
                end
                coeffsPIDout = setfield(coeffsPIDout,cfieldname,dati);
            end
        end
    end
end


