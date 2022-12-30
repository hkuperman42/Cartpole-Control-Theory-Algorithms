function [mask] = getCircleMask(xDim, yDim, xCenter, yCenter, radius)
%getCircleMask Generates a mask of a circle

%Preallocate space for the mask
mask = zeros(yDim, xDim, 'logical');

%Define the angles we need to cover
theta = linspace(0, 2*pi, 4 * pi * radius);

%Get x and y vectors for each point along the circumference.
x = radius * cos(theta) + xCenter;
y = radius * sin(theta) + yCenter;

%Assemble those coordinates into a mask
for k = 1 : length(x)
    row = round(y(k));
    col = round(x(k));
    if col > 0
        mask(row, col) = 1;
    end
end
