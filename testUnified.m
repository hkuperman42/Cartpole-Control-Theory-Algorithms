%Generate a vector to specify the constants of the system
constants = [1, 5, 0.25, 9.8, 0.025];

%Store the number of steps in the time horizon
numSteps = 80;

%Generate the cost matrices
Q = [0.1, 0, 0, 0;
     0, 0.1, 0, 0;
     0, 0, 0.1, 0;
     0, 0, 0, 0.01];
Sn = [10, 0, 0, 0;
     0, 10, 0, 0;
     0, 0, 10, 0;
     0, 0, 0, 10];

%Generate the control cost function
R = 1/10000;

%Set the initial position of x
x0 = [0, 0, pi, 0];

%Set the convergence threshold for iLQR and MPC
cThresh = 0.5;

%Test the controls against these initial conditions
testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0, 0], cThresh, 100, 10, 0, [1, 1, 1], " Control");









%Vary the starting position
%x0(3) = pi / 2;
%testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " SP: 0.5pi");
%x0(3) = 0.75;
%testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " SP: 0.75");
%x0(3) = 0;
%testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " SP: 0");
%x0(3) = pi;

%Vary the time horizon
%testControls(constants, Q, R, Sn, x0, 100, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " TH: 1sec");
%testControls(constants, Q, R, Sn, x0, 500, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " TH: 5sec");
%testControls(constants, Q, R, Sn, x0, 1000, [0, 0, 0, 0], cThresh, 100, 10, [1, 1, 1], " TH: 10sec");

%Vary the linearization point of LQR
%testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, pi / 2, 0], cThresh, 100, 10, [1, 0, 0], " Lin: 0.5pi");
%testControls(constants, Q, R, Sn, x0, numSteps, [0, 0, pi, 0], cThresh, 100, 10, [1, 0, 0], " Lin: pi");

%Vary the convergence threshold of MPC and ILQR
%testControls(constants, Q, R, Sn, x0, 100, [0, 0, 0, 0], 0.001, 100, 10, [0, 1, 1], " CT: 0.001");
%testControls(constants, Q, R, Sn, x0, 500, [0, 0, 0, 0], 0.1, 100, 10, [0, 1, 1], " CT: 0.1");

%Vary the time horizon of MPC
%testControls(constants, Q, R, Sn, x0, 100, [0, 0, 0, 0], cThresh, 150, 10, [0, 1, 1], " MPCTH: 150");
%testControls(constants, Q, R, Sn, x0, 500, [0, 0, 0, 0], cThresh, 200, 10, [0, 1, 1], " MPCTH: 200");