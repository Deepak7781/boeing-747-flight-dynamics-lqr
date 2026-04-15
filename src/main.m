ac = boeing747Params();
[lonSS, latSS] = stateSpaceModel(ac);

Alon = lonSS.A;

% Compute the eigenvalues of the state matrix Alon
lonEigen = eig(Alon);

Alat = latSS.A;

latEigen = eig(Alat);

