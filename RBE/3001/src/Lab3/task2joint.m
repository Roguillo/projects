%% 
clc; clear;

% Create a robot object
stormtrooper = Robot();
travel_time  = 2;

% Define three [x, y, z, gamma] positions for the end effector
% These positions are located on the x-z plane of the robot
v_1_joint  = [0 -68.8184  57.8320 -71.3672];
v_2_joint  = [0 -12.8320 -26.1914 -52.6465];
v_3_joint  = [0  16.3477  -9.0527 -46.2305];

% Store individual joint angles
v_joints   = [v_1_joint; v_2_joint; v_3_joint];

% Compute FK
v_1_fkmat  = stormtrooper.fk_3001(deg2rad(v_1_joint));
v_2_fkmat  = stormtrooper.fk_3001(deg2rad(v_2_joint));
v_3_fkmat  = stormtrooper.fk_3001(deg2rad(v_3_joint));

% Get task space coordinates
v_1_task   = [v_1_fkmat(1, 4), v_1_fkmat(2, 4), v_1_fkmat(3, 4)];
v_2_task   = [v_2_fkmat(1, 4), v_2_fkmat(2, 4), v_2_fkmat(3, 4)];
v_3_task   = [v_3_fkmat(1, 4), v_3_fkmat(2, 4), v_3_fkmat(3, 4)];

% Gamma values
gamma_1    = deg2rad(v_1_joint(2) + v_1_joint(3) + v_1_joint(4));
gamma_2    = deg2rad(v_2_joint(2) + v_2_joint(3) + v_2_joint(4));
gamma_3    = deg2rad(v_3_joint(2) + v_3_joint(3) + v_3_joint(4));

% Triangl edge trajectories
traj_1_mat = [TrajGenerator.cubic_traj(v_1_task(1), v_2_task(1), 0, travel_time);
              TrajGenerator.cubic_traj(v_1_task(2), v_2_task(2), 0, travel_time);
              TrajGenerator.cubic_traj(v_1_task(3), v_2_task(3), 0, travel_time);
              TrajGenerator.cubic_traj(gamma_1, gamma_2, 0, travel_time)];
              
traj_2_mat = [TrajGenerator.cubic_traj(v_2_task(1), v_3_task(1), 0, travel_time);
              TrajGenerator.cubic_traj(v_2_task(2), v_3_task(2), 0, travel_time);
              TrajGenerator.cubic_traj(v_2_task(3), v_3_task(3), 0, travel_time);
              TrajGenerator.cubic_traj(gamma_2, gamma_3, 0, travel_time)];

traj_3_mat = [TrajGenerator.cubic_traj(v_3_task(1), v_1_task(1), 0, travel_time);
              TrajGenerator.cubic_traj(v_3_task(2), v_1_task(2), 0, travel_time);
              TrajGenerator.cubic_traj(v_3_task(3), v_1_task(3), 0, travel_time);
              TrajGenerator.cubic_traj(gamma_3, gamma_1, 0, travel_time)];

% Concatentate trajectories
traj_mats  = {traj_1_mat, traj_2_mat, traj_3_mat};

% Move to first vertex to start
interpolate_time = 2;  
stormtrooper.interpolate_jp(v_1_joint, interpolate_time);
pause(interpolate_time);

% Create plot variables
sack_o_joints  = zeros(4, 1);  
measure_buffer = zeros(2, 4); 

j              = 1;

% Triangle loop
for i = 1:3
    vertex = traj_mats{i}; % Goal vertex
    tic;
    
    while toc < travel_time
        % Collect joint data
        measure_buffer      = stormtrooper.measure_js(true, false);
        sack_o_joints(:, j) = measure_buffer(1, :)';
        j                   = j + 1;

        % Evaluate the trajectory at the current time
        traj = TrajGenerator.eval_traj(vertex, toc);
        
        % Extract task-space coordinates (x, y, z, gamma) from the trajectory
        x     = traj(3);
        y     = traj(7);
        z     = traj(11);
        gamma = traj(15);
        
        % Calculate joint angles using inverse kinematics
        spinnies = rad2deg(stormtrooper.ik3001([x, y, z, gamma]));
        
        % Move the robot to the calculated joint position
        stormtrooper.interpolate_jp(spinnies, 0.015); % 15 ms is about how long measure_js() takes, so we use it here for interpolate
        pause(0.015);
    end
end

% Create coordinate matrix and buffer FK matrix
sack_o_coords = zeros(3, 1);
fk_mat_buffer = zeros(4, 4);

% Convert to task space
for i = 1:width(sack_o_joints)
    fk_mat_buffer = stormtrooper.fk_3001(deg2rad(sack_o_joints(:, i)));
    sack_o_coords(:, i) = [fk_mat_buffer(1, 4), fk_mat_buffer(2, 4), fk_mat_buffer(3, 4)];
end

% Plot both task and joint space trajectories
subplot(2, 1, 1);
scatter3(sack_o_coords(1, :), sack_o_coords(2, :), sack_o_coords(3, :));
title('Task Space');
xlabel('x'); ylabel('y'); zlabel('z');
axis equal;

subplot(2, 1, 2);
scatter3(sack_o_joints(2, :), sack_o_joints(3, :), sack_o_joints(4, :));
title('Joint Space');
xlabel('\theta_2'); ylabel('\theta_3'); zlabel('\theta_4');
axis equal;