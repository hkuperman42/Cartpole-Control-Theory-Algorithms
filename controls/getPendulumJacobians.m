function [Jx, Ju] = getPendulumJacobians(constants)
%getJacobians Generates the jacobians of the pendulum system

%Seperate the constants passed into the function
[mQ, mC, l, g, T] = splitFive(constants);

%Generate several symbolic variables for the purposes of using MATLAB's
%built-in linearization function
syms x dx theta dtheta u;

%For ease of notation, store some commonly used combinations of constants
M = mQ + mC;
ml = mQ * l;

%Calculate theta double-dot and x double-dot
d2theta = (M*g*sin(theta) - ml*dtheta*dtheta*sin(theta)*cos(theta) - u*cos(theta)) / (4*M*l/3 - ml*cos(theta)*cos(theta));
d2x = (u + ml*dtheta*dtheta*sin(theta) - ml*d2theta*cos(theta)) / M;

%Generate a function that simulates the cart pole dynamics (this is
%incredibly messy)
f(1) = x + dx*T; %+ 0.5*d2x*T*T;
f(2) = dx + d2x*T;
f(3) = theta + dtheta*T; %+ 0.5*d2theta*T*T;
f(4) = dtheta + d2theta*T;

%Generate the jacobian of the dynamics function
Jx = jacobian(f, [x, dx, theta, dtheta]);
Ju = jacobian(f, u);

%Generate a new symbolic expression that doesn't clash with the other
%variable names
syms xL dxL thetaL dthetaL uL;
Jx = subs(Jx, [x, dx, theta, dtheta, u], [xL, dxL, thetaL, dthetaL, uL]);
Ju = subs(Ju, [x, dx, theta, dtheta, u], [xL, dxL, thetaL, dthetaL, uL]);
end