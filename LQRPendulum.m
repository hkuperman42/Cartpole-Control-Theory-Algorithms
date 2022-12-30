function [K] = LQRPendulum(Q, R, Sn, linState, numSteps, Jx, Ju)
%hunterLQR Finds the time-varying Kalman gains matrix to determine the
%control inputs to the system at a particular time

%Generate the A and B matrices, linearizing around the given state
syms xL dxL thetaL dthetaL uL;

%Create a vector of variables to substitute
newSub(1:4) = linState;
newSub(5) = 0;

%Linearize the dynamics of the system
A = vpa(subs(Jx, [xL, dxL, thetaL, dthetaL, uL], newSub), 16);
B = vpa(subs(Ju, [xL, dxL, thetaL, dthetaL, uL], newSub), 16);

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

end