function [polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder)

%%
d2r = pi/180;
r2d = 1/d2r;


%%
% fit the pwm and angle data
polyCoef = polyfit(angle_deg*d2r, pwm_us, fitOrder);

% Evaluate the fit accross the full PWM range
angleEval_deg = -45:1:45;
pwmEval_us = polyval(polyCoef, angleEval_deg*d2r);

% Compute the residuals 
pwmResid_us = pwm_us - polyval(polyCoef, angle_deg*d2r);

% Find the Zero angle command
pwmZero_us = polyval(polyCoef, 0.0*d2r);

% Plot
figure;
subplot(2,1,1)
plot(pwm_us, angle_deg, '*k', pwmEval_us, angleEval_deg, 'r', pwmZero_us, 0, 'ob'); grid on;
ylabel('angle (deg)')
subplot(2,1,2)
plot(pwm_us, pwmResid_us, '*k'); grid on;
ylabel('PWM (us)'); ylabel('PMW Resiual (us)');
xlim([900, 2100]);
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

% Print out the coefficients for Copy-Paste into the config.h file
fprintf(1, '\n{ ');
for indx = 1:length(polyCoef)
    fprintf(1, '%.8f', polyCoef(indx));
    if indx < length(polyCoef)
        fprintf(1, ',\t');
    end
end
fprintf(1, '}\n');

