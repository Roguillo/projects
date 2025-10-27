ip = ImageProcessor(debug=true);
bot = Robot();

% uv = [474, 233];
% 
% p_robot = ip.image_to_robot(uv)
Nokia            = Eye_of_Sauron.cam;

frame            = Eye_of_Sauron.getImage(); imshow(frame);
[x, y]           = ginput(1);
x = x-17;
y = y-12.5;
p_img            = [x(1), y(1)];
% p_img_ul         = [x(1), y(1)]; % upper left  corner (camera pov)
% p_img_ur         = [x(2), y(2)]; % upper right corner (camera pov)
% p_img_ll         = [x(3), y(3)]; % lower left  corner (camera pov)
% p_img_lr         = [x(4), y(4)]; % lower right corner (camera pov)
p_img_c          = [960, 540]; % center             (camera pov)
p_img_ball       = p_img       ; % filler to be replaced

%height of the camera
h = 190; %in mm

%robot frames
p_0_c            = Government_drone.image_to_robot(p_img_c);
p_0_ball         = Government_drone.image_to_robot(p_img_ball);

z                = Eye_of_Sauron.cam_T(3);
u_checker        = p_0_ball(1) - p_0_c(1);
v_checker        = p_0_ball(2) - p_0_c(2);
theta            = asin(h/z);

r                = [sqrt(z^(2) + v_checker^(2) - 2*z*v_checker*cos(theta));
                    ];


%%lucas' experimental code
theta2 = asin(h,r); %the diagonal line thru the centroid


p_0_ball_real    = [p_0_ball(1) + (sqrt(r^(2) - h^(2)))/(h)*cos((u_checker)/(r)), ...
                    p_0_ball(2) + (sqrt(r^(2) - h^(2)))/(h)*sin((u_checker)/(r))
                    ];


