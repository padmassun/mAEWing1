%% Geri AFS Bode-Gain Schedule

% Check: at 35 m/sec, KB = 0.0015, at 36 m/sec, KB = 0.0020 rad/ft/sec^2
% TAS in m/sec, Accel measurements in ft/sec^2, KB in rad/ft/sec^2

%m2f=3.28084;

if TAS<=34, KB=0.0010; end
    
if TAS>34, KB=(0.0005*TAS-0.016); end

% If an output scale factor is used to implememt the gain schedule, the 
% controller output should be multiplied by the following scale factor (SF)

if TAS<=34, SF=1; end

if TAS>34, SF=(0.5*TAS-16.); end

% Note: Predicted closed-loop flutter speed is 36.7 m/sec TAS