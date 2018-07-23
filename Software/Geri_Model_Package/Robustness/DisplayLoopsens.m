function [loops,Peak] = DisplayLoopsens(G,C,w,option)
% [loops,Peak] = DisplayLoopsens(G,C,w,option)
%
%               performs the LOOPSENS command and displays the closed loop 
%               transfer functions
%   loops : structure generated from LOOPSENS
%   Peaks : structure containing the Hinf-norms of the closed loop transfer
%           functions
%        ---
%       option = 
%       'g' for singular value plot of the gang-of-six
%       'ui' for unstructured multiplicative input uncertainty (cross-coupling allowed)
%       'uo' for unstructured multiplicative output uncertainty (cross-coupling allowed)
%       'ua' for unstructured additive uncertainty (cross-coupling allowed)
%       'si' for structured multiplicative input uncertainty (no cross-coupling)
%       'so' for structured multiplicative output uncertainty (no cross-coupling)

% Robustness Interpretation from Zhou, Doyle, Glover 95
% T  : output (sensor) errors, neglected HF dynamics, changing # of rhp zeros
% Ti : input (actuators) errors, neglected HF dynamics, changing # of rhp zeros
% S  : LF parameter errors, changing # of rhp poles
% Si : LF parameter errors, changing # of rhp poles
% KS : additive plant errors, neglected HF dynamics, uncertain rhp zeros
% SG : LF parameter errors, neglected HF dynamics, uncertain rhp poles & zeros
%
% Signal Interpretation
% T  : noise, reference signal
% S  : output disturbance
% KS : control effort
% SG : input disturbance




if nargin==3
    option = 'g';
end
[ny, nu] = size(G);
loops = loopsens(G,C);

Peak.So  = norm(loops.So,'inf');
Peak.PSi = norm(loops.PSi,'inf');
Peak.CSo = norm(loops.CSo,'inf');
Peak.Ti  = norm(loops.Ti,'inf');
Peak.To  = norm(loops.To,'inf');
Peak.Si  = norm(loops.Si,'inf');

loops.So.InputName = G.OutputName; loops.So.OutputName = G.OutputName;
loops.Si.InputName = G.InputName; loops.Si.OutputName = G.InputName;
loops.To.InputName = G.OutputName; loops.To.OutputName = G.OutputName;
loops.Ti.InputName = G.InputName; loops.Ti.OutputName = G.InputName;
loops.PSi.InputName = G.InputName; loops.PSi.OutputName = G.OutputName;
loops.CSo.InputName = G.OutputName; loops.CSo.OutputName = G.InputName;

        b = bodeoptions;
        b.MagUnits = 'abs';
        

switch lower(option)
    case 'g'
    figure
    % Gang-of-Six Singular Value Plot
    YL = -40;
    subplot(3,2,1)
        sigma(loops.So,w);grid;ylim([YL(1),db(2*Peak.So)]);
        title('So') % title('S')
    subplot(3,2,2)
        sigma(loops.PSi,w);grid;ylim([YL(1),db(2*Peak.PSi)]);
        title('PSi') % title('SG')
    subplot(3,2,3)
        sigma(loops.CSo,w);grid;ylim([YL(1),db(2*Peak.CSo)]);
        title('CSo')
    subplot(3,2,4)
        sigma(loops.Ti,w);grid;ylim([YL(1),db(2*Peak.Ti)]);
        title('Ti')
    subplot(3,2,5)
        sigma(loops.To,w);grid;ylim([YL(1),db(2*Peak.To)]);
        title('To')
    subplot(3,2,6)
        sigma(loops.Si,w);grid;ylim([YL(1),db(2*Peak.Si)]);
        title('Si')
    if exist('garyfyFigure','file');garyfyFigure; end
        
    case 'si'
    % Multiplicative Structured Uncertainty Input
    for ii=1:nu
    figure
        [MAG,~,W] = bode(loops.Ti(ii,ii),b,w);
        cleanMAG = squeeze(MAG);
        cleanMAG(abs(cleanMAG)<1e-9) = 1e-9;
        area(W,1./cleanMAG);grid;
        ylim([0,2]);xlim([w{1},w{2}]);
        set (gca, 'Xscale', 'log');
        ylabel('Magnitude (abs)')
        title(['Allowed multiplicative uncertainty at input ' int2str(ii) ' ("' G.InputName{ii} '")'])
        if exist('garyfyFigure','file');garyfyFigure; end
    end
    
    case 'ui'
    % Multiplicative Unstructured Uncertainty Input
    figure
        [SV, W] = sigma(loops.Ti,w);
        area(W,1./SV(1,:));grid; 
        ylim([0,2]); xlim([w{1},w{2}]);
        set (gca, 'Xscale', 'log');
        ylabel('Singular Values (abs)')
        title('Allowed unstructured multiplicative uncertainty at inputs')
        if exist('garyfyFigure','file');garyfyFigure; end

        
    
    case 'so'
    % Multiplicative Uncertainty Output
    for ii=1:ny
    figure
        [MAG,~,W] = bode(loops.To(ii,ii),b,w);
        cleanMAG = squeeze(MAG);
        cleanMAG(abs(cleanMAG)<1e-9) = 1e-9;
        area(W,1./cleanMAG);grid;
        ylim([0,2]);xlim([w{1},w{2}]);
        set (gca, 'Xscale', 'log');
        ylabel('Magnitude (abs)')
        title(['Allowed multiplicative uncertainty at output ' int2str(ii) ' ("' G.OutputName{ii} '")'])
        if exist('garyfyFigure','file');garyfyFigure; end
    end
    
    
    case 'uo'
    % Multiplicative Uncertainty Output
    figure
        [SV, W] = sigma(loops.To,w);
        area(W,1./SV(1,:));grid;ylim([0,2]);xlim([w{1},w{2}]);
        ylabel('Singular Values (abs)')
        set (gca, 'Xscale', 'log');
        title('Allowed unstructured multiplicative uncertainty at outputs')
        if exist('garyfyFigure','file');garyfyFigure; end
    
    case 'ua'
    figure        
    % Additive Uncertainty
    [SV, W] = sigma(loops.CSo,w);
        area(W,1./SV(1,:));grid;ylim([0,10]);xlim([w{1},w{2}]);
        ylabel('Singular Values (abs)')
        set (gca, 'Xscale', 'log');
        title('Allowed additive plant uncertainty')
        if exist('garyfyFigure','file');garyfyFigure; end

end

