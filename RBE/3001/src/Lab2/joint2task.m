clc;

% Create a robot object
robot = Robot();

% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!

% robot.measure_js(true, false)

%to get these points, we manually moved the robot to our desired positions
%and recorded each point of interest.
v_1 = [0 -68.8184  57.8320 -71.3672];
v_2 = [0   8.8770 -61.0840 -19.6875];
v_3 = [0  65.2148 -58.5352 -90.9668];

% Move your robot to the first vertex
% robot.servo_jp(v_1);


% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
vertices = [v_1; v_2; v_3]; %matrix of our coordinate matrices
bag_o_joints = [0 0 0 0]; % where we will record our joint positions
bag_o_timestamps = [0]; %where we will record timestamps
time = 2; %2 seconds between each movement 

j = 1; %index
tic;  % start timer
for i=1:3 %iterating between all 3 movements
    % See lab_demo.m from lab 1 for inspiration on how to move your robot
    % and collect data
    robot.interpolate_jp(vertices(mod(i,3) + 1, :), time); %move to the coordinates we specified in each index of vertices

    while toc/i < time %while between each movement,
        measurement = robot.measure_js(true, false); %record positions
        bag_o_joints(j, :) = measurement(1, :); %store positions
        bag_o_timestamps(j) = toc; %store timestamp
        j = j + 1; %increment
    end
end

bag_o_timestamps = bag_o_timestamps.';

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

coords = [0 0 0]; %initialize coordinate matrix
for i = 1:length(bag_o_joints) %iterate thru all of the joint positions
    fk_mat = robot.fk_3001(deg2rad(bag_o_joints(i, :))); %record frame coords
    coords(i, :) = [fk_mat(1, 4) fk_mat(2, 4) fk_mat(3, 4)]; %store frame coords
end

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html


%plots
subplot(2, 1, 1);
scatter3(coords(:, 1), coords(:, 2), coords(:, 3));
title('Task Space');
xlabel('x');
ylabel('y');
zlabel('z');
axis equal

subplot(2, 1, 2);
scatter3(bag_o_joints(:, 2), bag_o_joints(:, 3), bag_o_joints(:, 4));
title('Joint Space');
xlabel('q_2');
ylabel('q_3');
zlabel('q_4');
axis equal

% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
