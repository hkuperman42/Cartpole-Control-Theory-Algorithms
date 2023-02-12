function [I] = generateAnimationCartPole(constants, numSteps, states, u, fileName)
%generateAnimationPendulum Generates an animation of the cart pole control

%Normalize the position-values of state so we don't go off-screen
[s1, s2] = size(states);
comp = ones(1, s2);
states(1, :) = min(states(1, :), 1.74 * comp);
states(1, :) = max(states(1, :), -1.74 * comp);

%Determine whether we need to skip frames (due to MATLAB writing memory constraints)
toStep = floor((s2 - 1) / 200) + 1;

%Preallocate space for the gif
I = zeros(900, 1600, 1, round(numSteps / toStep), "logical");

%Actually animate the cart
for i = 1:toStep:s2
    frame = zeros(900, 1600, "logical");

    %Draw the track
    frame(580, :) = 255;

    %Draw the cart
    leftX = round(400 * states(1, i) + 701);
    rightX = round(400 * states(1, i) + 899);
    frame(540, leftX:rightX) = 1;
    frame(490, leftX:rightX) = 1;
    frame(490:540, leftX) = 1;
    frame(490:540, rightX) = 1;

    %Draw the wheels
    frame(getCircleMask(1600, 900, leftX + 20, 540 + 20, 20)) = 1;
    frame(getCircleMask(1600, 900, rightX - 20, 540 + 20, 20)) = 1;

    %Draw the length of the pendulum
    length = 400 * constants(3);
    for j = 1:length
        x = round(states(1, i) * 400 + 800 + j * sin(states(3, i)));
        y = round(515 - j * cos(states(3, i)));
        frame(y, x) = 1;
    end

    %Draw the weight on the end of the pendulum
    length = length + 30;
    circleX = round(states(1, i) * 400 + 800 + length * sin(states(3, i)));
    circleY = round(515 - length * cos(states(3, i)));
    frame(getCircleMask(1600, 900, circleX, circleY, 30)) = 1;

    %Draw a force vector
    center = round(states(1, i) * 400) + 800;
    length = abs(round(u(min(i, s1 - 1)) * 2.5));
    if states(2, i) < 0
        frame(465:470, max(1, center - length):center) = 1;
    else 
        frame(465:470, center:min(center + length, 1600)) = 1;
    end

    %Make the lines thicker
    frame = imdilate(frame, strel('square', 2));

    %Write the frame to an image
    I(:, :, 1, floor((i - 1) / toStep) + 1) = frame;
end

%Comcatenate all frames into one gif
Image = ones(900, 1600, 1, round(s2 / toStep)) * 255;
Image(I) = 0;
imwrite(Image, fileName, "GIF", "DelayTime", constants(5) * toStep);