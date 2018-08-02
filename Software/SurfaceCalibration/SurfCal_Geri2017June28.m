%% L4
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = -[-24.43  -19.28  -13.41  -6.94  -1.43  0.04  1.21  7.12  14.55  22.34  29.64  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);

%% L3
% pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
% angle_deg = -[-30.97  -24.09  -17.11  -9.61  -3.52  -1.90  -0.43  5.68  13.51  21.44  29.58  ]; % Ensure Inclinometer and AC convention are the same!!

pwm_us = [1120.00  1220.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ] - 20;
angle_deg = -[-30.97  -24.09  -9.61  -3.52  -1.90  -0.43  5.68  13.51  21.44  29.58  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);

%% L2
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = -[-28.87  -22.21  -15.44  -7.69  -1.62  0.01  1.23  7.49  15.09  22.91  31.05  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);

%% L1
% pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
% angle_deg = -[-25.50  -19.89  -15.46  -7.97  -1.84  -0.41  0.87  6.87  14.35  22.14  30.08  ]; % Ensure Inclinometer and AC convention are the same!!

pwm_us = [1120.00  1220.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = -[-25.50  -19.89  -1.84  -0.41  0.87  6.87  14.35  22.14  30.08  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);

%% R1
pwm_us = [1920.00  1820.00  1720.00  1620.00  1540.00  1520.00  1500.00  1420.00  1320.00  1220.00  1120.00  ];
angle_deg = -[-19.83  -15.54  -10.99  -5.99  -1.45  -0.18  1.16  5.90  12.48  19.59  26.78  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R2
pwm_us = [1920.00  1820.00  1720.00  1620.00  1540.00  1520.00  1500.00  1420.00  1320.00  1220.00  1120.00  ];
angle_deg = -[-26.05  -20.52  -14.08  -7.31  -1.51  -0.57  1.33  7.39  15.37  23.28  31.54  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R3
pwm_us = [1920.00  1820.00  1720.00  1620.00  1540.00  1520.00  1500.00  1420.00  1320.00  1220.00  1120.00  ];
angle_deg = -[-24.98  -19.45  -13.35  -7.64  -1.42  -0.04  1.46  7.12  15.30  23.29  31.80  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R4
pwm_us = [1920.00  1820.00  1720.00  1620.00  1540.00  1520.00  1500.00  1420.00  1320.00  1220.00  1120.00  ];
angle_deg = -[-24.56  -19.16  -13.13  -6.64  -1.18  0.13  1.51  7.38  15.22  23.40  31.82  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


