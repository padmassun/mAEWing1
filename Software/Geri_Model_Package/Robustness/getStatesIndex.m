function StatesIndex  = getStatesIndex(StateNames,SelectedStateNames)
% StatesIndex = getStatesIndex(StateNames,SelectedStateNames) 
%   
%         StateNames: Cell-Array with all StateNames of the model
% SelectedStateNames: Cell-Array with Names of Selected States
%
%        StatesIndex: Vector of indices of states that are to be kept

StatesIndex = [];
ll = 1;
for ii = 1 : size(StateNames,1)
    kk = 0;
    for jj = 1 : length(SelectedStateNames)
        i_x = strcmp(StateNames{ii},SelectedStateNames{jj});
        if i_x
            kk = kk + 1;
        else
        end
    end
    if kk ~= 0;
        StatesIndex(ll) = ii;
        ll = ll+1;
    else
    end
end
end

