clc; clear;

% Move your robot to a point where Y=0, X > 150 and Z > 150

Trooper   = Robot();                              % Robot object
period    = 5;                                    % Time period
threshold = 500;

start_p = [0 -200 250 0]                          % starting point task space vector
start_q = Trooper.ik3001(start_p);                % starting point joint space vector

Trooper.interpolate_jp(rad2deg(start_q), period); % get to starting point
pause(period);

% Generate a trajectory that will move your robot in a straight line from
% your current (X, Y, Z) to the point (X, -Y, Z)

goal_p    = [0 200 250 0]                                                 % end point task space vector
goal_q    = Trooper.ik3001(goal_p);                                       % end point joint space vector

coeff_mat = [TrajGenerator.cubic_traj(start_p(1), goal_p(1), 0, period);  % coefficients matrix for all joints
             TrajGenerator.cubic_traj(start_p(2), goal_p(2), 0, period);
             TrajGenerator.cubic_traj(start_p(3), goal_p(3), 0, period);
             TrajGenerator.cubic_traj(start_p(4), goal_p(4), 0, period)]; 

%% Execute the trajectory
tic;                       % Start timer

p_buff      = zeros(1, 4); % buffer row vector for holding task space point
q_buff      = zeros(1, 4); % buffer row vector for holding joint space values
q_read_buff = zeros(2, 4); % buffer matrix for holding joint measurements
fk_mat_buff = zeros(4)   ; % buffer fk matrix
gamma_buff  = 0          ; % buffer gamma value

p_d         = zeros(4, 1); % 3xn matrix to hold task space trajectory points
q           = zeros(4, 1); % 4xn matrix to hold joint space measurements
p           = zeros(4, 1); % 4xn matrix to hold task space measurements
J_dets      = zeros(1, 1); % 1xn vector to hold Jacobian determinant values

j           = 1;

while toc < period
    % Evaluate your trajectory, and send your robot to its next setpoint
    path        = TrajGenerator.eval_traj(coeff_mat, toc);           % evaluate trajectory

    p_buff      = [path(3), path(7), path(11), path(15)];            % get next point
    p_d(:, j)     = p_buff';

    q_buff      = rad2deg(Trooper.ik3001(p_buff));                   % convert to joint space

    Trooper.interpolate_jp(q_buff, 0.015);                           % move to next point
    pause(0.015);

    % Record the joint position of your robot 
    q_read_buff = deg2rad(Trooper.measure_js(true, false));          % measure joint values
    q(:, j)     = q_read_buff(1, :)';                                % store joint values
    fk_mat_buff = Trooper.fk_3001(q_read_buff);                      % get fk matrix
    gamma_buff  = q_read_buff(2) + q_read_buff(3) + q_read_buff(4);  % get gamma
    p(:, j)     = [fk_mat_buff(1:3, 4); gamma_buff];                 % get measured task space values

    % Record the determinant of the Jacobian
    J           = Trooper.jacob3001(q_read_buff(1, :));              % get Jacobian
    J_dets(j)   = det(J(1:3, :)*J(1:3, :)');                         % get and store determinant

    JpXTranspose = J(1:3, :)*J(1:3,:)';
    
    J_dets(j)   = det(J(1:3, :)*J(1:3, :)');                % get and store determinant
    J_CNs(j) = max(eig(JpXTranspose))/min(eig(JpXTranspose));



    j           = j + 1;





    % Test if your robot is too close to a singularity, and stop if it is
    if Trooper.atSingularity(q_read_buff(1, :), threshold)
        break
    end
end

figure(1)
plot(0:j-2, J_dets)
figure(2)
plot(0:j-2, J_CNs)