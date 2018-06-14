(B. Danowsky, STI, 5/21/2018)

This is meant to be a standalone package that contains all of the MATLAB functions, scripts, and data used to generate an aeroelastic state space model of the mAEWing1 vehicle Geri.

Actuator model as TF objects: GeriActuators.mat
	contains servo actuator: G_surface_actuator
	contains 25 ms Pade delay model: Delay_25ms [added 6/14/2018]
	TODO [6/14/2018]: need to add engine model
Sensor models as TF objects: Geri_SensorModels.mat

Run: GenerateModel_DEMO.m
Script will generate a state space model as a function of freestream velocity with the following outputs, inputs, states, and corresponding units. 

Outputs:

    ' 1 : u           [ft/s  ]'
    ' 2 : alpha       [rad   ]'
    ' 3 : theta       [rad   ]'
    ' 4 : q           [rad/s ]'
    ' 5 : h           [ft    ]'
    ' 6 : beta        [rad   ]'
    ' 7 : phi         [rad   ]'
    ' 8 : p           [rad/s ]'
    ' 9 : r           [rad/s ]'
    '10 : qcg         [rad/s ]'
    '11 : pcg         [rad/s ]'
    '12 : nzcg        [ft/s^2]'
    '13 : nzCBaft     [ft/s^2]'
    '14 : nzCBfwd     [ft/s^2]'
    '15 : nzRwingaft  [ft/s^2]'
    '16 : nzLwingaft  [ft/s^2]'
    '17 : nzRwingfwd  [ft/s^2]'
    '18 : nzLwingfwd  [ft/s^2]'
    '19 : delBFS      [rad   ]'
    '20 : delIBS      [rad   ]'
    '21 : delmidS     [rad   ]'
    '22 : delOBS      [rad   ]'
    '23 : delT lb     [lb    ]'
    '24 : delBFAS     [rad   ]'
    '25 : delIBAS     [rad   ]'
    '26 : delmidAS    [rad   ]'
    '27 : delOBAS     [rad   ]'
    '28 : hdot        [ft/s  ]'
    '29 : Right Twist [ft/s^2]'
    '30 : Left Twist  [ft/s^2]'
    '31 : Asym Twist  [ft/s^2]'


Inputs:

    ' 1 : Sym BF  [rad ]'
    ' 2 : Sym IB  [rad ]'
    ' 3 : Sym Mid [rad ]'
    ' 4 : Sym OB  [rad ]'
    ' 5 : Thrust  [lb  ]'
    ' 6 : AS BF   [rad ]'
    ' 7 : AS IB   [rad ]'
    ' 8 : AS Mid  [rad ]'
    ' 9 : AS Ob   [rad ]'
    '10 : wGust   [ft/s]'


States:

    ' 1 : u      [ft/s ]'
    ' 2 : w      [ft/s ]'
    ' 3 : theta  [rad  ]'
    ' 4 : q      [rad/s]'
    ' 5 : h      [ft   ]'
    ' 6 : beta   [rad  ]'
    ' 7 : phi    [rad  ]'
    ' 8 : p      [rad/s]'
    ' 9 : r      [rad/s]'
    '10 : eta1   [-    ]' % modal states
    '11 : eta2   [-    ]'
    '12 : eta3   [-    ]'
    '13 : eta4   [-    ]'
    '14 : eta5   [-    ]'
    '15 : eta6   [-    ]'
    '16 : eta1dt [1/s  ]' % modal state derivatives
    '17 : eta2dt [1/s  ]'
    '18 : eta3dt [1/s  ]'
    '19 : eta4dt [1/s  ]'
    '20 : eta5dt [1/s  ]'
    '21 : eta6dt [1/s  ]'