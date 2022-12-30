function [u, xOld, KSequence] = iLQRPendulum2(constants, Q, R, Sn, x0, cThresh, numSteps, startU)
%iLQRPendulum Generates the control sequence for each timestep based on
%inputted cost matrices for an inverted pendulum situation

%Store the size of the final cost matrix to make set-up easier
[sn1, sn2] = size(Sn);

%Preallocate a vector of zeros for the first-pass controls and the states
%along with vectors to hold the old states
u = startU;
x = zeros(sn2, numSteps, 'double');

%Set the initial state of x
x(:, 1) = x0;

%Preallocate space for v and S
v = zeros(sn1, numSteps, 'double');
S = zeros(sn1, sn2, numSteps, 'double');

%Initalize a constant to hold the cost and a matrix to hold the final K's
j = 0;
KSequence = zeros(1, sn2, numSteps, 'double');

%Complete the first forwards pass
for i = 2:numSteps
    %Generate the next state of the system using the real dynamics
    x(:, i) = simulateCartPole(constants, x(:, i - 1), u(i - 1));

    %Add the additional cost
    j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*u(i - 1)*R*u(i - 1);
end

%Add the cost for the final state
j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

%Prep for the first iteration
jOld = Inf();

%Prep for linearization
syms xL dxL thetaL dthetaL uL;
[Jx, Ju] = getPendulumJacobians(constants);

numIterations = 0;

%Loop until convergence
while (abs(j - jOld) > cThresh) && (numIterations < 50)
    %Prepare for the next forwards pass
    v(:, numSteps) = Sn*x(:, numSteps);
    S(:, :, numSteps) = Sn;

    %Complete a backwards pass
    for i = (numSteps - 1) : -1 : 1
        %Create a vector of variables to substitute
        newSub(1:4) = x(:, i);
        newSub(5) = u(i);

        %Linearize the dynamics of the system
        A = vpa(subs(Jx, [xL, dxL, thetaL, dthetaL, uL], newSub), 16);
        B = vpa(subs(Ju, [xL, dxL, thetaL, dthetaL, uL], newSub), 16);

        %Generate the control sequence for the current step
        Unified = B.' * S(:, :, i + 1) * B + R;
        K = Unified \ (B.' * S(:, :, i + 1) * A);
        Kv = Unified \ B.';
        Ku = Unified \ R;
    
        %Use these to generate the previous S and v values
        S(:, :, i) = A.' * S(:, :, i + 1) * (A - B*K) + Q;
        v(:, i) = (A - B*K).'*v(:, i + 1) - K.'*R*u(i) + Q*x(:, i);

        %Generate the change in the optimal control
        du(i) = -1 * (Kv*v(:, i + 1) + Ku*u(i));

        %Store the sequence of K matrices
        KSequence(:, :, i) = K;
    end

    %Prepare for the next interation
    xOld = x;
    uOld = u;
    jOld = j;
    j = 0;
    
    %Preform a pseudo line-search to prevent the control from blowing up
    numHalves = 0;
    while numHalves < 5
        %Simulate the next forwards pass
        for i = 2:numSteps
            %Generate the new optimal control
            dx = x(:, i - 1) - xOld(:, i - 1);
            dx(3) = mod((pi + dx(3)), 2*pi) - pi;
            u(i - 1) = u(i - 1) + du(i - 1) - KSequence(:, :, i - 1)*dx;

            %Generate the next state of the system using the real dynamics
            x(:, i) = simulateCartPole(constants, x(:, i - 1), u(i - 1));
            x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;
    
            %Add the additional cost
            j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*u(i - 1)*R*u(i - 1);
        end

        %Add the cost for the final state
        j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

        %If the new cost is less than the old one, continue to the next
        %iteration. If not, reset using a smaller du
        if j < jOld
            break;
        else
            u = uOld;
            x = xOld;
            j = 0;
            du = 0.5 * du;
            numHalves = numHalves + 1;
        end
    end

    %Increment the number of iterations
    numIterations = numIterations + 1;
end