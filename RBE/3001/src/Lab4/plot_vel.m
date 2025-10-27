clc; clear;

Trooper   = Robot();                              % Robot object
period    = 2;                                    % Time period
threshold = 3.85e+12;

q_vals  = [0 -68.8184  57.8320 -71.3672;
           0 -12.8320 -26.1914 -52.6465;
           0  16.3477  -9.0527 -46.2305];
p_vals  = zeros(3, 4);

fk_mat_buff  = zeros*(4);
gamma_buff   = 0;

for i = 1:3
    fk_mat_buff  = Trooper.fk_3001(deg2rad(q_vals(i, :)));
    gamma_buff   = deg2rad(q_vals(i, 2) + q_vals(i, 3) + q_vals(i, 4));
    p_vals(i, :) = [fk_mat_buff(1:3, 4); gamma_buff]';
end

Trooper.interpolate_jp(q_vals(1, :), period); % get to starting point
pause(period);

coeff_book = zeros(4, 4, 3);

for i = 1:3
    for j = 1:4
        coeff_book(j, :, i) = TrajGenerator.cubic_traj(p_vals(i, j), p_vals(mod(i, 3) + 1, j), 0, period);
    end
end

p_buff      = zeros(1, 4); % buffer row vector for holding task space point
q_buff      = zeros(1, 4); % buffer row vector for holding joint space values
q_read_buff = zeros(2, 4); % buffer matrix for holding joint measurements
vels_buff   = zeros(3, 1);

q           = zeros(4, 1); % 4xn matrix to hold joint space measurements
J_dets      = zeros(1, 1); % 1xn vector to hold Jacobian determinant values
measured_vels        = zeros(3, 1); % 3xn matrix to hold end-effector linear velocities
goal_vels   = zeros(3, 1);


j           = 1;

for i = 1:3
    tic;                       % Start timer
    
    while toc < period
        path        = TrajGenerator.eval_traj(coeff_book(:, :, i), toc);  % evaluate trajectory
    
        p_buff      = [path(3), path(7), path(11), path(15)];   % get next point
        q_buff      = rad2deg(Trooper.ik3001(p_buff));          % convert to joint space
    
        Trooper.interpolate_jp(q_buff, 0.015);                  % move to next point
        pause(0.015);
    
        q_read_buff = deg2rad((Trooper.measure_js(true, true)));
        q(:, j)     = q_read_buff(1, :)';                       % store joint values
    
        J           = Trooper.jacob3001(q_read_buff(1, :));     % get Jacobian
        
        J_dets(j)   = det(J(1:3, :)*J(1:3, :)');                % get and store determinant
        j           = j + 1;
    
        if Trooper.atSingularity(q_read_buff(1, :), threshold)
            break
        end
    
        measured_vels_buff   = Trooper.dk3001(q_read_buff(1, :), q_read_buff(2, :));
        measured_vels(:, j)  = measured_vels_buff(1:3)';

        goal_vels(:, j) = [path(4); path(8); path(12)];
    end
end

plot(1:j, [measured_vels(:, :); goal_vels(:, :)])
title('linear-velocity vs. time')
xlabel('time')
ylabel('linear-velocity')