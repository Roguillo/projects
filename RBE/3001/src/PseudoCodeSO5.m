Pseudocode for lab 5 signoff 5

bot = Robot();

z_hover = z; %the z value at which we hover over the ball and drop it in the baskets. z can be anything
z_ball = z; %the z value at which we pick up the ball. z can be anything

%assume we have the location of the ball in robot coordinates, as well as
%the location of all four of the ball recepticles, and a home position. x y are different
%for all of these locations, and z is predetermined by z_hover and z_ball
ball_location = [x,y, z_ball];
red_basket = [x,y, z_hover];
orange_basket = [x,y, z_hover];
yellow_basket = [x,y, z_hover];
green_basket = [x,y, z_hover];
home_posn = [x, y, z_hover];

%this is the color of the ball, which will be used to determine which of
%the baskets the ball will go into. This will be found using an external
%function.
ball_color = color;

bot.open_gripper();
bot.blocking_ts_move([ball_location, pi/2]); % pi/2 is the orientation of the end effector (gamma)
bot.close_gripper();

if ball_color == red
    bot.blocking_ts_move([red_basket, pi/2]); % Move to the red basket
    bot.open_gripper(); % Release the ball into the red basket
elseif ball_color == orange
    bot.blocking_ts_move([orange_basket, pi/2]); % Move to the orange basket
    bot.open_gripper(); % Release the ball into the orange basket
elseif ball_color == yellow
    bot.blocking_ts_move([yellow_basket, pi/2]); % Move to the yellow basket
    bot.open_gripper(); % Release the ball into the yellow basket
elseif ball_color == green
    bot.blocking_ts_move([green_basket, pi/2]); % Move to the green basket
    bot.open_gripper(); % Release the ball into the green basket
end

% Move back to the home position after placing the ball
bot.blocking_ts_move([home_posn, pi/2]);



