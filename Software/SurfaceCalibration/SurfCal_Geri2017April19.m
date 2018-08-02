%% L1
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [27.62  20.72  14.74  8.21  2.86  1.83  0.84  -5.30  -13.19  -20.88  -28.73  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% L2
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [29.13  22.53  15.60  7.89  1.85  0.17  -1.25  -7.40  -15.02  -22.81  -30.89  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% L3
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [28.74  21.62  14.44  6.87  0.87  -0.77  -2.23  -8.33  -16.17  -24.21  -32.42  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% L4
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [28.56  22.41  15.85  8.75  3.05  1.64  0.03  -5.98  -13.34  -20.91  -28.42  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R1
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [-26.96  -19.61  -12.40  -5.45  -0.48  0.84  1.88  6.85  12.14  16.95  21.16  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R2
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [-32.18  -23.89  -15.96  -8.04  -1.86  -0.32  0.96  6.75  13.53  20.14  26.39  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R3
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [-34.76  -26.21  -18.10  -9.83  -3.87  -2.33  -1.03  4.69  11.62  18.18  24.08  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);


%% R4
pwm_us = [1120.00  1220.00  1320.00  1420.00  1500.00  1520.00  1540.00  1620.00  1720.00  1820.00  1920.00  ];
angle_deg = [-34.07  -25.35  -17.09  -8.99  -2.80  -1.19  0.21  6.17  13.20  19.55  25.43  ]; % Ensure Inclinometer and AC convention are the same!!
fitOrder = 2; % Start with quadratic, alter as required
[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);

