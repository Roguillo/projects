%% USE THIS FILE TO GET SIGN OFF 3

robot = Robot();  % Create a robot object to use
interpolate_time = 5000000; % TODO: choose a reasonable time to interpolate

%% Send the robot to its 0 position
% We don't need to collect data yet, as we're just putting the robot at its
% starting point

% YOUR CODE HERE

%% Send the robot to an arbitrary position
% pos1 = [a b c d];  % TODO: Choose a joint position

% TODO: Initialize an array full of 0s to store the positions
% See https://www.mathworks.com/help/matlab/ref/zeros.html
% We do this because allocating memory takes a lot of time, so we only want
% to do it once

% Initialize another array for the timestamps (or use the same array as the
% positions, your call)

% TODO: Call a function to move the robot

% Collect data as the robot moves
tic;  % Start timer
while toc < interpolate_time
    % Read current joint positions (not velocities though!)

    % Store the positions and timesetamps in an array 
    % Hint: use 'toc' to get the current elapsed time since tic
end

%% Make your figure
% To use subfigures, you'll use the subplots feature of MATLAB.
% https://www.mathworks.com/help/matlab/ref/subplot.html

% In each subplot you create, you can use 'plot' to plot one joint value vs
% time
% https://www.mathworks.com/help/matlab/ref/plot.html

% Remember titles, labels, and units!

%% Calculate time step statistics
% MATLAB may have some functions for the requested statistics...