function testControls(constants, Q, R, Sn, x0, numSteps, linState, cThresh, MPCHorizon, MPCSteps, stateNoise, measNoise, toRun, nameExtension)
%testControls tests all of the controls using the specified conditions and
%generates gif animations of their performance

%Generate the Jacobians of the system so we don't have to do it every time
%we run LQR. We do also do this again in ILQR, but I'm too lazy to change
%it atm
[Jx, Ju] = getPendulumJacobians(constants);

%For consistancy
rng('default');
stateNoise = normrnd(0, 1, 4, numSteps) .* repmat(stateNoise.', 1, numSteps);
measNoise = normrnd(0, 1, 4, numSteps) .* repmat(measNoise.', 1, numSteps);

%Check if we need to run LQR
if toRun(1) == 1
    %Run LQR on the pendulum system
    [K, uExp] = LQRPendulum(constants, Q, R, Sn, linState, x0, numSteps, Jx, Ju);

    %RUN LQR USING JUST FF
    %Set the initial state of x
    x = zeros(4, numSteps, 'double');
    x(:, 1) = x0;

    %Simulate the real dynamics of system
    j = 0;
    u = uExp;
    for i = 2:numSteps
        %Generate the next state of the system using the real dynamics
        x(:, i) = simulateCartPole(constants, x(:, i - 1), u(i - 1)) + stateNoise(4, i);
        x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;

        %Add the additional cost
        j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*u(i - 1)*R*u(i - 1);
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)
    
    %Animate the cart pole using LQR
    if j < 50000
        generateAnimationCartPole(constants, numSteps, x, u, "FFLQR Cartpole" + string(nameExtension) + " (" + string(round(j)) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "FFLQR Cartpole" + string(nameExtension) + " (Failed).gif", "GIF");
    end


    %RUN LQR USING THE GAINS MATRIX
    %Set the initial state of x
    x = zeros(4, numSteps, 'double');
    x(:, 1) = x0;
    xMeas = x(:, 1);

    %Simulate the real dynamics of system
    j = 0;
    u = zeros(1, numSteps);
    for i = 2:numSteps
        %Generate the control for the next step
        u(i - 1) = -K(:, :, i - 1)*(xMeas);
    
        %Generate the next state of the system using the real dynamics
        x(:, i) = simulateCartPole(constants, x(:, i - 1), u(i - 1)) + stateNoise(4, i);
        x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;

        %Generate the noise in the state measurement
        xMeas = x(:, i) + measNoise(4, i);

        %Add the additional cost
        j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*u(i - 1)*R*u(i - 1);
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)
    
    %Animate the cart pole using LQR
    if j < 50000
        generateAnimationCartPole(constants, numSteps, x, u, "LQR Cartpole" + string(nameExtension) + " (" + string(round(j)) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "LQR Cartpole" + string(nameExtension) + " (Failed).gif", "GIF");
    end
end



%Check whether we need to run ILQR
if toRun(2) == 1
    %Run ILQR on the pendulum system
    [uOpt, xOpt, K] = iLQRPendulum2(constants, Q, R, Sn, x0, cThresh, numSteps, zeros(numSteps, 1, 'double'));

    %RUN ILQR USING JUST FF
    %Reset the initial state of x
    x = zeros(4, numSteps, 'double');
    x(:, 1) = x0;

    %Simulate the real dynamics of system
    j = 0;
    for i = 2:numSteps
        %Generate the next state of the system using the real dynamics
        x(:, i) = simulateCartPole(constants, x(:, i - 1), uOpt(i - 1)) + stateNoise(4, i); 
        x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;
    
        %Add the additional cost
        j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*uOpt(i - 1)*R*uOpt(i - 1);
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

    %Animate the cart pole using ILQR
    if j < 5000000
        generateAnimationCartPole(constants, numSteps, x, uOpt, "FFILQR Cartpole" + nameExtension + " (" + round(j) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "FFILQR Cartpole" + nameExtension + " (Failed).gif");
    end




    %RUN ILQR USING THE GAINS MATRIX
    %Reset the initial state of x
    x = zeros(4, numSteps, 'double');
    x(:, 1) = x0;
    xMeas = x0.';

    %Simulate the real dynamics of system
    j = 0;
    for i = 2:numSteps
        %Generate the new optimal control
        xMeas
        dx = xMeas - xOpt(:, i - 1);
        dx
        dx(3) = mod((pi + dx(3)), 2*pi) - pi;
        uOpt(i - 1) = uOpt(i - 1) - K(:, :, i - 1)*dx;
    
        %Generate the next state of the system using the real dynamics
        x(:, i) = simulateCartPole(constants, x(:, i - 1), uOpt(i - 1)) + stateNoise(4, i); 
        x(3, i) = mod((pi + x(3, i)), 2*pi) - pi;

        %Generate the noise in the state measurement
        xMeas = x(:, i) + measNoise(4, i);
    
        %Add the additional cost
        j = j + 0.5*x(:, i - 1).'*Q*x(:, i - 1) + 0.5*uOpt(i - 1)*R*uOpt(i - 1);
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

    %Animate the cart pole using ILQR
    if j < 5000000
        generateAnimationCartPole(constants, numSteps, x, uOpt, "ILQR Cartpole" + nameExtension + " (" + round(j) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "ILQR Cartpole" + nameExtension + " (Failed).gif");
    end
end



%Check whether we need to run MPC
if toRun(3) == 1
    %RUN MPC USING LQR
    %Begin by resetting the initial state of x and creating a variable to hold u 
    x = zeros(4, numSteps, 'double');
    u = zeros(1, numSteps, 'double');
    x(:, 1) = x0;

    %Store the initial value
    x0Real = x0;

    %Iterate through the horizon, generating a new control policy at
    %each set of MPCSteps steps
    j = 0;
    for l = 1:MPCSteps:numSteps
        %Determine the length of the horizon in this iteration
        horizonLength = min(MPCHorizon, numSteps - l - 1);

        %Generate the new control policy for this set of steps
        x0 = x(:, l);
        [K, uOpt] = LQRPendulum(constants, Q, R, Sn, x0, x0, numSteps, Jx, Ju);
        
        %Simulate the real dynamics of system
        [x1, x2] = size(xOpt);
        numStepsThisIteration = min(MPCSteps, x2);
        for i = 2:(numStepsThisIteration + 1)
            %Generate the control for the next step
            u(i - 1) = -K(:, :, i - 1)*xMeas;
    
            %Generate the next state of the system using the real dynamics
            x(:, l + i - 1) = simulateCartPole(constants, x(:, l + i - 2), uOpt(i - 1)) + stateNoise(4, i); 
            x(3, l + i - 1) = mod((pi + x(3, l + i - 1)), 2*pi) - pi;
    
            %Add the additional cost
            j = j + 0.5*x(:, l + i - 2).'*Q*x(:, l + i - 2) + 0.5*uOpt(i - 1)*R*uOpt(i - 1);
        end
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

    %Animate the cart pole using MPC
    if j < 5000000
        generateAnimationCartPole(constants, numSteps, x, u, "MPC Cartpole" + nameExtension + " (" + round(j) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "MPC Cartpole" + nameExtension + " (Failed).gif");
    end



    %RUN MPC USING ILQR
    %Shift the convergence threshold to be more lenient
    cThresh = cThresh * 10;

    %Begin by resetting the initial state of x and creating a variable to hold u 
    x = zeros(4, numSteps, 'double');
    u = zeros(1, numSteps, 'double');
    x(:, 1) = x0Real;

    %Generate a fake uOopt for the first iteration
    uOpt = zeros(1, MPCSteps, 'double');

    %Iterate through the horizon, generating a new control policy at
    %each set of MPCSteps steps
    j = 0;
    for l = 1:MPCSteps:numSteps
        %Determine the length of the horizon in this iteration
        horizonLength = min(MPCHorizon, numSteps - l - 1);

        %Use the previous control policy (if it exists) as a guess for the
        %next run of ILQR
        [u1, u2] = size(uOpt);
        guessU = zeros(1, MPCHorizon, 'double');
        guessU(1:(u2 - MPCSteps)) = uOpt((MPCSteps + 1):u2);

        %Generate the new control policy for this set of steps
        x0 = x(:, l);
        [uOpt, xOpt, K] = iLQRPendulum2(constants, Q, R, Sn, x0, cThresh, horizonLength, guessU);
        
        %Simulate the real dynamics of system
        [x1, x2] = size(xOpt);
        numStepsThisIteration = min(MPCSteps, x2);
        for i = 2:(numStepsThisIteration + 1)
            %Generate the new optimal control
            dx = x(:, l + i - 2) - xOpt(:, i - 1) + measNoise(4, i);
            dx(3) = mod((pi + dx(3)), 2*pi) - pi;
            uOpt(i - 1) = uOpt(i - 1) - K(:, :, i - 1)*dx;
    
            %Generate the next state of the system using the real dynamics
            x(:, l + i - 1) = simulateCartPole(constants, x(:, l + i - 2), uOpt(i - 1)) + stateNoise(4, i); 
            x(3, l + i - 1) = mod((pi + x(3, l + i - 1)), 2*pi) - pi;

            %Store the control at this location
            u(l + i - 2) = uOpt(i - 1);
    
            %Add the additional cost
            j = j + 0.5*x(:, l + i - 2).'*Q*x(:, l + i - 2) + 0.5*uOpt(i - 1)*R*uOpt(i - 1);
        end
    end

    %Add the cost for the final state
    j = j + 0.5*x(:, numSteps).'*Sn*x(:, numSteps)

    %Animate the cart pole using MPC
    if j < 5000000
        generateAnimationCartPole(constants, numSteps, x, u, "MPC Cartpole" + nameExtension + " (" + round(j) + ").gif");
    else 
        imwrite(ones(900, 1600, 'double'), "MPC Cartpole" + nameExtension + " (Failed).gif");
    end
end