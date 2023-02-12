%Generate a vector to specify the constants of the system
constants = [1, 5, 0.25, 9.8, 0.025];

%Store the number of steps in the time horizon
numSteps = 80;

%Generate the cost matrices and starting position
Q = [0.1, 0, 0, 0;
     0, 0.1, 0, 0;
     0, 0, 0.1, 0;
     0, 0, 0, 0.01];
Sn = [10, 0, 0, 0;
     0, 10, 0, 0;
     0, 0, 10, 0;
     0, 0, 0, 10];
R = 1/10000;
x0 = [0, 0, pi, 0];

%Set the convergence threshold for iLQR and MPC
cThresh = 0.25; 

%Set the noise to zero
noise = [0, 0, 0, 0];

%Test the controls against these baseline initial conditions
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, pi, 0], cThresh, 30, 1, 1, noise, noise, [1, 1, 1], " Control");

%Test the controls with low noise
sig = 0.01;
noise = [1, 1, 1, 1];
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, pi, 0], cThresh, 30, 1, sig, noise, noise, [1, 1, 1], " Low Noise");

%Test the controls with high noise
sig = 0.025;
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, pi, 0], cThresh, 30, 1, sig, noise, noise, [1, 1, 1], " High Noise");

%Change the initial state
x0 = [0, 0, 0.6, 0];
noise = [0, 0, 0, 0];
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0.6, 0], cThresh, 30, 1, 1, noise, noise, [1, 1, 1], " 0.6lin Control");

%Test the controls with low noise
sig = 0.01;
noise = [1, 1, 1, 1];
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0.6, 0], cThresh, 30, 1, noise, noise, [1, 1, 1], " 0.6lin Low Noise");

%Test the controls with high noise
sig = 0.025;
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0.6, 0], cThresh, 30, 1, noise, noise, [1, 1, 1], " 0.6lin High Noise");