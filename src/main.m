ac = boeing747Params();
[lonSS, latSS] = stateSpaceModel(ac);

Alon = lonSS.A;

% Compute the eigenvalues of the state matrix Alon
lonEigen = eig(Alon);

Alat = latSS.A;

latEigen = eig(Alat);

t = 0:0.1:50;

x0 = [0.1 0 0 0];

[y,tout,x] = initial(lonSS, x0, t);


figure;
plot(tout, x);
xlabel('Time (s)');
ylabel('Output Response');
title('System Response of the Boeing 747 Model');
legend('u', '\alpha', 'q' ,'\theta', 'Location', 'Best');
grid on;

% Display the eigenvalues for both longitudinal and lateral dynamics
disp('Eigenvalues of the longitudinal state matrix:');
disp(lonEigen);
disp('Eigenvalues of the lateral state matrix:');
disp(latEigen);

% Transfer function
% Input : Elevator Deflection
% Output : Pitch Angle
lonTF = tf(lonSS);

elevPitchTF = lonTF(4);

step(elevPitchTF)