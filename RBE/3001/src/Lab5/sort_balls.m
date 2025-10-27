clc; clear;

% In this file, write your "main" script for picking up the balls
% This should only be 10-15 lines long, the vast majority of the work
% should be in your Robot and ImageProcessor classes

% Ultimately it's up to you to decide how to organize this, but the
% "detect_balls" and "pick_up_ball" functions should do a lot of the work
% for you.

% @@@@@@@@@@@@@@
% YOUR CODE HERE
% @@@@@@@@@@@@@@

bot     = Robot();
IP      = ImageProcessor(debug = true);
cam_obj = IP.camera;
lens    = cam_obj.cam;
%preview(lens);


% debugging purposes

%bw = IP.generate_static_mask();
% figure(1);
%imshow(bw);
frame         = cam_obj.getImage();
%imshow(frame);
%[x,y] = ginput(4)


[bot_arr, colors]   = IP.detect_balls();

while true

    for i = 1:height(bot_arr)

        bot.pick_up_ball(bot_arr(i, :), colors(1, i));



        if i == height(bot_arr)
            pause(5);
            [bot_arr, colors] = IP.detect_balls();


        end

        while isempty(bot_arr)
            disp("No balls found. Place more balls on board");
            pause(5);
            [bot_arr, colors] = IP.detect_balls();
        end



    end


end
% while true
% 
%     for i = 1:height(bot_arr)
% 
%         bot.pick_up_ball(bot_arr(i, :), colors(1, i));
% 
%         if i == height(bot_arr)
%             pause(5);
%             [bot_arr, colors] = IP.detect_balls();
% 
%         end
% 
%     end




%end

