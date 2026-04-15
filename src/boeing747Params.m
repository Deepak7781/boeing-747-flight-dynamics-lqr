function boeing747Params()

% Aircraft Parameters
ac = struct();

ac.alt = 20000; %ft
ac.mach = 0.65;
ac.g = 32.174; % ft/s2
ac.tas = 673; % ft/s
ac.rho = 0.0407; % lb/ft3
ac.dynPres = 287.2; % lb/ft2
ac.weight = 636636; % lb
ac.S = 5500; % ft2
ac.b = 196; % ft
ac.c = 27.3; % ft
ac.cg = 0.25; % 25% of c
ac.alpha = 2.5; % deg
ac.Ixx = 1.82e7; % slug-ft2
ac.Iyy = 3.31e7; % slug-ft2
ac.Izz = 4.97e7; % slug-ft2
ac.Ixz = -4.05e5; % slug-ft2

% Longitudinal Derivatives
ac.Xu = -0.0059; % 1/s
ac.Xalpha = 15.9787; % ft/s2
ac.Zu = -0.1104; % 1/s
ac.Zalpha = -353.52; % ft/s2
ac.Mu = 0.0000; % 1/ft.s
ac.Malpha = -1.3028; % 1/s2
ac.Malphadot = -0.1057; % 1/s
ac.Mq = -0.5417; % 1/s
ac.XdelE = 0; % ft/s2
ac.ZdelE = -25.5659; % ft/s2
ac.MdelE = -1.6937; % 1/s2

% Lateral-Directional Derivatives
ac.Ybeta = -71.9142; % ft/s2
ac.Lbeta = -2.7255; % 1/s2
ac.Lp = -0.8434; % 1/s
ac.Lr = 0.3224; % 1/s
ac.Nbeta = 0.9962; % 1/s2
ac.Np = -0.0236; % 1/s
ac.Nr = -0.2539; % 1/s
ac.YdelR = 9.5872; % ft/s2
ac.LdelR = 0.1363; % 1/s2
ac.NdelR = -0.6226; % 1/s2
ac.YdelA = 1.0386; % ft/s2
ac.LdelA = 0.2214; % 1/s2
ac.NdelA = 0.0112; % 1/s2

end