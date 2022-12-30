function [xNext] = simulateCartPole(constants, xCurr, uCurr)
%simulateCartPole Simulates one discrete time step of the cartpole
%dynamical system using a modified (?) Euler's method

%Seperate the constants passed into the function
[mQ, mC, l, g, T] = splitFive(constants);

%Seperate the x inputs
[x, dx, theta, dtheta] = splitFour(xCurr);

%For ease of notation, store some commonly used combinations of constants
M = mQ + mC;
ml = mQ * l;

%Calculate theta double-dot and x double-dot
d2theta = (M*g*sin(theta) - ml*dtheta*dtheta*sin(theta)*cos(theta) - uCurr*cos(theta)) / (4*M*l/3 - ml*cos(theta)*cos(theta));
d2x = (uCurr + ml*dtheta*dtheta*sin(theta) - ml*d2theta*cos(theta)) / M;

%Generate the next state vector of the system
xNext(1) = x + dx*T; %+ 0.5*d2x*T*T;
xNext(2) = dx + d2x*T;
xNext(3) = theta + dtheta*T; %+ 0.5*d2theta*T*T;
xNext(4) = dtheta + d2theta*T;
end