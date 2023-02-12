function [K, u] = LQRPendulum(constants, Q, R, Sn, linState, x0, numSteps, Jx, Ju)
%hunterLQR Finds the time-varying Kalman gains matrix to determine the
%control inputs to the system at a particular time

%Generate the A and B matrices, linearizing around the given state
syms xL dxL thetaL dthetaL uL;

%Create a vector of variables to substitute
newSub(1:4) = linState;
newSub(5) = 0;

%Linearize the dynamics of the system
A = double(subs(Jx, [xL, dxL, thetaL, dthetaL, uL], newSub));
B = double(subs(Ju, [xL, dxL, thetaL, dthetaL, uL], newSub));

%Store the size of the inputs
[s1, s2] = size(Sn);
[b1, b2] = size(B);

%Pre-emptively store the transpose of A and B and the inverse of R
At = A.';
Bt = B.';
Rinv = inv(R);

%Preallocate space for S and K
S = zeros(s1, s2, numSteps);
K = zeros(b2, b1, numSteps);

%Create a matrix to store all S-values
S(:, :, numSteps) = Sn;

%Loop for all timesteps, generating the S and K matrices
for k = numSteps - 1 : -1: 1
    S(:, :, k) = At * S(:, :, k + 1) * inv(B * Rinv * Bt * S(:, :, k + 1) + eye(s1, s2, 'double')) * A + Q;
    K(:, :, k) = inv(R + Bt * S(:, :, k + 1) * B) * Bt * S(:, :, k + 1) * A;
end

%Reset the initial state of x
x = zeros(4, numSteps, 'double');
x(:, 1) = x0;

%Simulate the real dynamics of system to generate the feed-forward-only control
u = zeros(1, numSteps);
for i = 2:numSteps
    %Generate the control for the next step
    u(i - 1) = -K(:, :, i - 1)*x(:, i - 1);
    
    %Generate the next state of the system using the real dynamics
    x(:, i) = simulateCartPole(constants, x(:, i - 1), u(i - 1));
    x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;
end