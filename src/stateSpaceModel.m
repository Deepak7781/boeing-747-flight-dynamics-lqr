function [lonSS, latSS] = stateSpaceModel(ac)
    
    % Longitudinal State-Space Model
    Alon = [ac.Xu ac.Xalpha 0 -ac.g;
            ac.Zu/ac.tas ac.Zalpha/ac.tas 1 0;
            ac.Mu+(ac.Malphadot*ac.Zu/ac.tas) ac.Malpha+(ac.Malphadot*ac.Zalpha/ac.tas) ac.Mq+ac.Malphadot 0;
            0 0 1 0];

    Blon = [ac.XdelE; ac.ZdelE/ac.tas; ac.MdelE+(ac.Malphadot*ac.ZdelE/ac.tas); 0];

    Clon = eye(4);
    Dlon = zeros(4,1);

    lonSS = ss(Alon, Blon, Clon, Dlon);

    % Lateral State-Space Model
    Alat = [ac.Ybeta/ac.tas 0 -1 ac.g/ac.tas;
            ac.Lbeta ac.Lp ac.Lr 0;
            ac.Nbeta ac.Np ac.Nr 0;
            0 1 0 0];
    Blat = [ac.YdelA/ac.tas ac.YdelR/ac.tas;
            ac.LdelA ac.LdelR;
            ac.NdelA ac.NdelR;
            0 0];

    Clat = eye(4);

    Dlat = zeros(4,2);

    latSS = ss(Alat, Blat, Clat, Dlat);
    
end