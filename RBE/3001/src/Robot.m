% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
    end


    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
        %SERVO_JP Send robot to a joint configuration
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
            
            % YOUR CODE HERE
            % Hint: look at lab1_base.m to see how to move your robot
            % Hint 2: don't set the travel time to 0, as that causes 
            %         ~unintended behaviors~; do something small like
            %         0.001

            travelTime = 0.001;
            tic; % begin software timer

            % moves the arms to the matrix q
            self.writeJoints(q);

            % Small travel time and display joint readings
            % while toc < travelTime
            %     disp(self.getJointsReadings);
            % end
        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
        %    t: a scalar that tells how long to take to travel to the new position
        %       in milliseconds

            % YOUR CODE HERE
            self.writeTime(time); % tells joints to move to their target positions in the amount of time specified
            self.writeJoints(q); % commands joints to move to their target positions, stored in q
        end

        function q_curr = measure_js(robot, GETPOS, GETVEL)
        %MEASURED_JS Get the current position and velocity of the robot
        % Inputs:
        %    GETPOS: a boolean indicating whether or not to retrieve joint
        %            positions
        %    GETVEL: a boolean indicating whether or not to retrieve joint
        %            velocities
        % Outputs:
        %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
        %            GETPOS is false), and whose bottom row contains joint 
        %            velocities
        
            % This line gets the current positions of the joints
            % (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            
            % This line gets the current joint velocities
            % (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            
            % YOUR CODE HERE
            % Hint: Only collect the data you need; serial read operations
            %       are expensive!
            q_curr = [0 0 0 0; 0 0 0 0]; % set q_curr to a 2x4 matrix full of zeroes, so that data can be updated inside it
            if GETPOS
                posn = (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
                q_curr = q_curr + [posn ; 0 0 0 0]; % Update the first row with joint positions
            end
            if GETVEL
                vels = (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
                q_curr = q_curr + [0 0 0 0 ; vels]; % Update the second row with joint velocities
            end
        end
        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
        %DH2MAT Gives the transformation matrix for a given row of a DH table
        % Inputs:
        %    dh_row: a [1x4] vector containing representing a single row of a DH
        %            table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        row

            % YOUR CODE HERE
            % Hint: Use the degree version of trig functions
                
            %this is our homogenous transfer function that is modeled after
            %the one given in class. We will use this to get from one frame
            %to the next.
            ht = [cos(dh_row(1)), -sin(dh_row(1))*cos(dh_row(4)),  sin(dh_row(1))*sin(dh_row(4)), dh_row(3)*cos(dh_row(1));
                  sin(dh_row(1)),  cos(dh_row(1))*cos(dh_row(4)), -cos(dh_row(1))*sin(dh_row(4)), dh_row(3)*sin(dh_row(1));
                               0,                 sin(dh_row(4)),                 cos(dh_row(4)),                dh_row(2);
                               0,                              0,                              0,                        1];
        end

        function ht = dh2fk(self, dh_tab)
        %DH2MAT_R Calculates FK from a DH table
        % Inputs:
        %    dh_tab: a [nx4] matrix representing a DH table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        table

            % YOUR CODE HERE
            % Hint: Recall from lecture that the full FK of a manipulator can
            % be obtained by post multiplying all the HT matrices from each row
            % of the DH table
    
            % Hint 2: You just wrote a function to calculate the HT matrix
            % corresponding to one row of the DH table
            i = 1; %index. Starts at 2 because we are using prevmatrix as the initial row
            prevmatrix = eye(4); %initializing prevmatrix as the first row coming from the DH table
            
            while (i <= height(dh_tab)) %iterating while i is less than the amount of rows of DH table
                ht = prevmatrix * self.dh2mat(dh_tab(i,:)); %multplying previous matrix with current dh row
                prevmatrix = ht; %updating output
                i = i+1; %incrementing
            end

        end

        function T = fk_3001(self, q) %takes 4 joint parameters into the fkMatrix_to_function function, which takes these parameters and converts them to 3D coordinates
            T = fkMatrix_to_function(q(1), q(2), q(3), q(4));
        end
        % YOUR CODE HERE
        % Write a function fk3001 that takes in an array of four joint
        % values and uses your matlabFunction'd FK to generate the FK of
        % your robot

        % Hint: All non-static methods (which this is) should take "self" as
        % their first argument

        %% END LAB 2 CODE
        function qs = ik3001(self, pos)
    %IK3001 Calculates IK for the OMX arm (radians)
    
    L_1   = 96.326; % (mm) Link 1
    L_2   = 130.23; % (mm) Link 2
    L_3   = 124;    % (mm) Link 3
    L_4   = 133.4;  % (mm) Link 4
    phi   = 0.185;  % (rad) offset angle on joint 2

    x_ee  = pos(1); % (mm)
    y_ee  = pos(2); % (mm)
    z_ee  = pos(3); % (mm)
    gamma = pos(4); % (rad)

    % Joint 1
    theta_1 = atan2(y_ee, x_ee);

    % Wrist center
    x = x_ee - L_4*cos(gamma)*cos(theta_1);
    y = y_ee - L_4*cos(gamma)*sin(theta_1);
    z = z_ee + L_4*sin(gamma);

    if z < 0
        z = 0;
    end

    A = sqrt(x^(2) + y^(2));

    r = [x; y];
    theta_1vector = [cos(theta_1); sin(theta_1)];

    dotproduct = dot(r, theta_1vector);

    if dotproduct < 0
        A = -A;
    end

    B = z - L_1;
    C = sqrt(A^2 + B^2);

    if C > (L_2 + L_3)
        C = 2*(L_2+L_3) - C;
    end

    % epsilon = acos((L_2^(2) + L_3^(2) - C^(2))/(2*L_2*L_3));
      epsilon = atan2(sqrt(1 - ((L_2^(2) + L_3^(2) - C^(2))/(2*L_2*L_3))^2), ...
                           (L_2^(2) + L_3^(2) - C^(2))/(2*L_2*L_3));
       beta    = atan2(B, A);
        % alpha   = acos((L_2^(2) + C^(2) - L_3^(2))/(2*L_2*C));
        alpha   = atan2(sqrt(1 - ((L_2^(2) + C^(2) - L_3^(2))/(2*L_2*C))^2), ...
                           (L_2^(2) + C^(2) - L_3^(2))/(2*L_2*C));

    theta_2 = pi/2 - (alpha + beta + phi);
    theta_3 = pi/2 - epsilon + phi;
    theta_4 = gamma - (theta_2 + theta_3);

    qs = [theta_1, theta_2, theta_3, theta_4];
end
        %% END LAB 3 CODE
        %% BEGIN LAB 4 CODE
        function j = jacob3001(self, qpos) 
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos

            j = FK_to_Jake(qpos(1), qpos(2), qpos(3), qpos(4));

        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular 
            %       velocity of the end effector in the base frame of the 
            %       robot
            vs = self.jacob3001(qpos) * qvel';
        end
            
        % YOUR CODE HERE: Write a function atSingularity as described in
        % the lab document
        function chicken_boolean = atSingularity(self, qpos, threshold)
            chicken_boolean = false;

            J = self.jacob3001(qpos);
            D = det(J(1:3, :)*J(1:3, :)');

            JpXTranspose = J(1:3, :)*J(1:3,:)';
            J_CNs = max(eig(JpXTranspose))/min(eig(JpXTranspose));
           
            if J_CNs > threshold
                chicken_boolean = true;
            end

        end

        %%%
        % computes IK numerically using Newton-Raphson method
        %
        % p_d :   4x1 vector for desired task space coordinates    (cartesian)
        % R   :   4x1 vector with corresponding joint space values (rad)
        %
        function R = ik_newt(self, p_d)
            err_th = 0.0001;        % error threshold
            lambda = 0.675;       % damping constant
    
            q      = zeros(4, 1); % 4x1 vector to hold and update joint space values per cycle
            p_g    = zeros(4, 1); % 4x1 vector to hold task space "guess" for comparison
            
            while norm(p_d - p_g) > err_th
                fk_mat   = self.fk_3001(q);                   % get FK matrix
                gamma    = q(2) + q(3) + q(4);                % get gamma value
                p_g      = [fk_mat(1:3, 4); gamma];           % get task space coordinates and assign to guess
            
                J        = self.jacob3001(q);                 % get Jake
                J        = [J(1:3, :); J(5, :)];              % mutilate Jake
                J        = J'*pinv(J*J' + lambda^(2)*eye(4)); % mutilate Jake further
                q_delta  = J*(p_d - p_g);                     % get joint space error
            
                q        = q + q_delta;                       % add joint space delta
            end
    
            R = q';                                           % get definitive joint space values
        end


    %% END LAB 4 CODE
    %% BEGIN LAB 5 CODE
    function blocking_js_move(self, qpos, nvargs)
        arguments
            self Robot;
            qpos double;
            nvargs.time double = 2;
        end
        %BLOCKING_JS_MOVE moves the robot to a position in joint space
        %before exiting
        % Inputs:
        %   qpos: a [1x4] matrix of joint positions (uses degrees. If you
        %   are feeding it radians, MAKE SURE TO CONVERT THEM FIRST)
        %   time: (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        
        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        

        self.interpolate_jp(qpos, nvargs.time); % move the robot
        pause(nvargs.time); %wait until the robot has finished moving

        %failsafe to ensure that the robot is finished moving
        velocities = [9,9,9,9;9,9,9,9]; %initialize velocities matrix

        while norm(velocities) > 0.01 %while the resultant vector of the values in velocities do not equal zero, repeat this loop
            velocities = measure_js(self, false, true); %obtain vels
            velocities = velocities(2,:); %isolate velocities
        end

    end

    function blocking_ts_move(self, pos, nvargs)
        arguments
            self Robot;
            pos double;
            nvargs.time double = 2;
            nvargs.mode string = "cubic"
        end
        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, alpha
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize

        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        
        %finding current joint space coordinates:
        q_j_0 = self.measure_js(true,false); %the positions of each joint, in degrees
        q_j_0 = q_j_0(1,:);

        %finding current task space coordinates:
        fk = self.fk_3001(deg2rad(q_j_0)); %computing FK of the robot using its joint positions
        q_t_0 = [fk(1,4), fk(2,4), fk(3,4), deg2rad(q_j_0(2) + q_j_0(3) + q_j_0(4))]; %task space -> x, y, z, and gamma
        
        
        %differentiating between creating cubic and quintic coefficients
        if nvargs.mode == "cubic"
            coeffs = [TrajGenerator.cubic_traj(q_t_0(1), pos(1), 0, nvargs.time);  %x
                      TrajGenerator.cubic_traj(q_t_0(2), pos(2), 0, nvargs.time);  %y
                      TrajGenerator.cubic_traj(q_t_0(3), pos(3), 0, nvargs.time);  %z
                      TrajGenerator.cubic_traj(q_t_0(4), pos(4), 0, nvargs.time);  %gamma
                     ];

        elseif nvargs.mode == "quintic"
            coeffs = [TrajGenerator.quinitic_traj(q_t_0(1), pos(1), 0, nvargs.time);  %x
                      TrajGenerator.quinitic_traj(q_t_0(2), pos(2), 0, nvargs.time);  %y
                      TrajGenerator.quinitic_traj(q_t_0(3), pos(3), 0, nvargs.time);  %z
                      TrajGenerator.quinitic_traj(q_t_0(4), pos(4), 0, nvargs.time);  %gamma
                     ];  

        else 
            error('mode given was not cubic or quintic. Check your spelling buddy'); 
        end

        %movement
        tic;  %start timer

        while toc < nvargs.time %until the interpolation time is reached,
            traj = TrajGenerator.eval_traj(coeffs, toc);    %evaluates the trajectory at the current time
            
            %grab the task space positions
            if nvargs.mode == "cubic"
                x     = traj(3);
                y     = traj(7);
                z     = traj(11);
                gamma = traj(15);
            elseif nvargs.mode == "quintic"
                x     = traj(4);
                y     = traj(10);
                z     = traj(16);
                gamma = traj(22);
            end

            % Calculate joint angles using inverse kinematics
            spinnies = rad2deg(self.ik3001([x, y, z, gamma]));

            % Move the robot to the calculated joint position
           self.interpolate_jp(spinnies, 0.01); % 10ms buffer
            pause(0.01);
        end


        %failsafe to ensure that the robot is finished moving
        velocities = [9,9,9,9;9,9,9,9]; %initialize velocities matrix

        while norm(velocities) > 0.01 %while the resultant vector of the values in velocities do not equal zero, repeat this loop
            velocities = measure_js(self, false, true); %obtain vels
            velocities = velocities(2,:); %isolate velocities
        end
    end

    % @@@@@@@@@@@@@@
    % YOUR CODE HERE
    % @@@@@@@@@@@@@@
    % Write a function pick_up_ball that satisfies the following
    % requirements:
    function pick_up_ball(self, pos, color, nvargs)
    %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
    % Inputs:
    %   pos: a [1x2] matrix representing the position of the ball in the XY
    %        frame of the robot
    %   color: a string indicating what color bin the ball should be placed
    %          in
    %   z_offset (optional): the z-position at which to begin the straight
    %                        vertical trajectory 
    %   z_ball (optional): the z-positon at which to stop the vertical 
    %                      trajectory and grasp the ball
    arguments
        self Robot;
        pos double;
        color string;
        nvargs.z_offset double = 100;
        nvargs.z_ball double = 20;
    end
    
        %getting rid of additional error we found
        pos = [pos(1)-6,pos(2)-3]
        ball_location = [pos, nvargs.z_ball]; %position of the ball

        %these x and y values were found by putting the robot arm in a
        %specifc position and converting measure_js values into task space
        %using fk
        red_basket = [6.67, -161, nvargs.z_offset]; % posn of red basket. it is at the bottom right corner looking at the board from behind the robot
        orange_basket = [96.4, -173.25, nvargs.z_offset]; % posn of orange basket. it is at the top right corner looking at the board from behind the robot
        yellow_basket = [74.42, 188.62, nvargs.z_offset]; % posn of yellow basket. it is at the bottom left corner looking at the board from behind the robot
        green_basket = [153.44, 180.10, nvargs.z_offset]; % posn of green basket. it is at the top left corner looking at the board from behind the robot
        home_posn = [153, 12.5, nvargs.z_offset];


        % Move to the home position
        self.blocking_ts_move([home_posn, pi/2], time=1.5);

        self.writeGripper(true); % open gripper  
        %hover over ball
        self.blocking_ts_move([ball_location(1:2), nvargs.z_offset, pi/2],time=1); % pi/2 is the orientation of the end effector (gamma)
        self.blocking_ts_move([ball_location, pi/2],time=1); %lower to ball

        self.writeGripper(false); % close gripper
        
        %sends the robot arm up to not interfere with the boxes
        self.blocking_ts_move([ball_location(1),ball_location(2), nvargs.z_offset, pi/3], time=1);

        if strcmp(color, "Red") %if the color of the ball is red,
           self.blocking_ts_move([red_basket, pi/4]); % Move to the red basket
           self.writeGripper(true); % Release the ball into the basket

        elseif strcmp(color, "Orange") %if the color of the ball is orange,
           self.blocking_ts_move([orange_basket, pi/4]); % Move to the orange basket
           self.writeGripper(true); % Release the ball into the basket

        elseif strcmp(color, "Yellow") %if the color of the ball is yellow,
           self.blocking_ts_move([yellow_basket, pi/4]); % Move to the yellow basket
           self.writeGripper(true); % Release the ball into the basket   

        elseif strcmp(color, "Green") %if the color of the ball is green,
           self.blocking_ts_move([green_basket, pi/4]); % Move to the green basket
           self.writeGripper(true); % Release the ball into the basket   
        end

        self.blocking_ts_move([home_posn, pi/2], time=1.5); % resets the robot back at the home position

    end

    end % end methods

end % end class 
