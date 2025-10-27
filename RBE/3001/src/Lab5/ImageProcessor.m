classdef ImageProcessor < handle
    properties 
        camera;  % The Camera object for this image processor
      
        % What other attributes does an ImageProcessor need?
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_Height; % The height of the camera
        IP_debug; % bool for if the debug is enabled
    end

    methods
        % @@@@@@
        % README: nvargs
        % @@@@@@
        % Throughout this file and the new functions in Robot.m, you will
        % see "nvargs" a lot. This stands for Name Value ARGuments. These
        % are much like kwargs in Python.

        % To pass a name-value argument to a function, run
        % function_name(position_arg1, position_arg2, name=value). Position 
        % arguments (like all the arguments you've used so far in this
        % course) always go before name value args. 

        % Name value arguments should always have a default value. This is
        % defined in the "arguments" field. If you don't pass a certain
        % name value argument to a function, it gets set as the default
        % value.

        % You can access the value of a name value argument via
        % "nvargs.argument_name"
        function self = ImageProcessor(nvargs)
            arguments
                nvargs.debug logical = false;
            end
            self.camera = Camera();  % Instantiate a Camera object
            % What else does image processor need to do when it is
            % instantiated? Are there any values you want to calculate once
            % to reuse again later?
            self.cam_IS = self.camera.getCameraIntrinsics();
            self.cam_R = self.camera.getRotationMatrix();
            self.cam_T = self.camera.getTranslationVector();
            self.cam_Height = 9999999999999999999999999999999999999; %CHANGE WHEN WE MEASURE IT
            self.IP_debug = nvargs.debug;

        end

        function mask = generate_static_mask(self, nvargs)
            arguments
                self ImageProcessor;
                nvargs.margin double = 10; % TODO: choose a default
            end
            %GENERATE_STATIC_MASK produces a binary mask that leaves only
            %the checkerboard and surrounding area visible
            % Inputs: 
            %   margin (optional): a number (what units?) indicating how
            %                      far from the edge of the checkerboard 
            %                      to keep in the mask
            % Outputs:
            %   mask: a binary mask that blacks out everything except a
            %         region of interest around the checkerboard

            % If self.debug == true, this function should capture a
            % picture, apply the mask, and display it

            % Note: poly2mask is a very helpful tool for this part
            % https://www.mathworks.com/help/images/ref/poly2mask.html

            % Note2: Camera.m offers a method for finding checkerboard
            % points. This includes the corners, which may be helpful...

           x = [349, 803, 896, 240]; % border points x-coordinates
           y = [245, 243, 392, 408]; % border points y-coordinates


           mask = poly2mask(x, y, 625, 1059);

           if self.IP_debug == true
               figure(1);
               imshow(mask);
           end

            

        end

        function p_robot = image_to_robot(self, uvpos)
            %IMAGE_TO_ROBOT transforms a point on the image to the
            %corresponding point in the frame of the robot
            % Inputs:
            %   uvpos: a [1x2] matrix representing an image (u, v) 
            %          coordinate
            % Outputs:
            %   p_robot: a [1x2] matrix representing the transformation of
            %            the input uvpos into the robot's base frame

            % YOUR CODE HERE

            p_checker   = pointsToWorld(self.camera.getCameraIntrinsics() , ...
                                        self.camera.getRotationMatrix()   , ...
                                        self.camera.getTranslationVector(), ...
                                        uvpos ...
                                        );          
            T_checker_0 = [0, 1,  0,  105 + p_checker(2);
                           1, 0,  0, -95  + p_checker(1);
                           0, 0, -1,  0                 ;
                           0, 0,  0,  1                 ];
            p_robot     = T_checker_0(1:2, 4);
        end

        function [colors, uv_centroids] = detect_centroids(self, image, nvargs)
            arguments
                self ImageProcessor;
                image uint8;
                nvargs.min_size int32 = 200;  % chooose a value
            end
            %DETECT_CENTROIDS detects the centroids of binary blobs of
            %large enough size
            % Iputs: 
            %   Image: an image of the environment that has already been
            %          masked for the environment and to isolate a single 
            %          color
            %   min_size (optional): the minimum size of a blob to consider
            %                        a ball
            % Outputs: 
            %   colors: a [1xn] matrix of strings indicating the color of
            %           the ball at each detected centroid
            %   uv_centroids: a [nx2] matrix of coordinates of valid
            %                 centroids in image coordinates

            % If self.debug == true, this function should display the image
            % that was passed to it with colored circles on it marking the
            % balls

            % If self.debug == true, this function should display the image
            % that was passed to it with each color mask applied
            % individually (4 figures total for this part).

            % YOUR CODE HERE

             picture = self.camera.getImage();
             bw = self.generate_static_mask();
             imgWCircles = picture;
             colors = [];

            [greenBW, ~] = greenMaskFINAL(picture);  % Mask only green balls
            [orangeBW, ~] = orangeMaskFINAL(picture); % Mask only orange balls
            [redBW, ~] = redMaskFINAL(picture);       % Mask only red balls
            [yellowBW, ~] = yellowMaskFINAL2(picture); % Mask only yellow balls


            % Masks each color mask with the checkerboard mask
            checkColorGreen = bw & greenBW;
            checkColorOrange = bw & orangeBW;
            checkColorRed = bw & redBW;
            checkColorYellow = bw & yellowBW;
           
            % Finds the centroids of each color ball and fill the array
            % "colors" so we know how many of each colored ball
            bwimageGreen = bwlabel(checkColorGreen);
            statsGreen = regionprops('table', bwimageGreen, 'Centroid', 'Area');
            statsGreen = statsGreen(statsGreen.Area > nvargs.min_size, :);
            uv_centroidsGreen = statsGreen.Centroid;
            if ~isempty(uv_centroidsGreen)
                for i = 1:size(uv_centroidsGreen, 1)
                    colors = [colors, {"Green"}];
                end
            end



            bwimageOrange = bwlabel(checkColorOrange);
            statsOrange = regionprops('table', bwimageOrange, 'Centroid', 'Area');
            statsOrange = statsOrange(statsOrange.Area > nvargs.min_size, :);
            uv_centroidsOrange = statsOrange.Centroid;
            if ~isempty(uv_centroidsOrange)
                for i = 1:size(uv_centroidsOrange, 1)
                    colors = [colors, {"Orange"}];
                end
            end


            bwimageRed = bwlabel(checkColorRed);
            statsRed = regionprops('table', bwimageRed, 'Centroid', 'Area');
            statsRed = statsRed(statsRed.Area > nvargs.min_size, :);
            uv_centroidsRed = statsRed.Centroid;
            if ~isempty(uv_centroidsRed)
                for i = 1:size(uv_centroidsRed, 1)
                    colors = [colors, {"Red"}];
                end
            end


            bwimageYellow = bwlabel(checkColorYellow);
            statsYellow = regionprops('table', bwimageYellow, 'Centroid', 'Area');
            statsYellow = statsYellow(statsYellow.Area > nvargs.min_size, :);
            uv_centroidsYellow = statsYellow.Centroid;
            if ~isempty(uv_centroidsYellow)
                for i = 1:size(uv_centroidsYellow, 1)
                    colors = [colors, {"Yellow"}];
                end
            end

            % Combines the centroids into one matrix
            uv_centroids = [uv_centroidsGreen;
                            uv_centroidsOrange;
                            uv_centroidsRed;
                            uv_centroidsYellow];


            


            if self.IP_debug == true

            shapeType = 'circle';
            lineWidth = 5;
            center = 1;


            % Places Green circle onto the image

            if ~isempty(uv_centroidsGreen)

                for i = 1:size(uv_centroidsGreen, 1)
                coordsGreen = [uv_centroidsGreen(i, 1), uv_centroidsGreen(i, 2)];
                propsGreen = regionprops(checkColorGreen, 'MajorAxisLength', 'MinorAxisLength');
                majorAxisGreen = propsGreen(1).MajorAxisLength;
                minorAxisGreen = propsGreen(1).MinorAxisLength;
                

            %diameterGreen = mean([majorAxisGreen, minorAxisGreen]);
            %radiusGreen = diameterGreen / 2;
                radiusGreen = 20;
                

                positionGreen = [coordsGreen(1), coordsGreen(2), radiusGreen];
                shapeColorGreen = 'green';

                %colors = [colors, {"Green"}];

                %Circle outline
                imgWCircles = insertShape(imgWCircles, shapeType, positionGreen, 'Color', shapeColorGreen, 'Linewidth', lineWidth);
                %Centroid
                imgWCircles = insertShape(imgWCircles, shapeType, [coordsGreen(1), coordsGreen(2), center], 'Color', shapeColorGreen, 'Linewidth', lineWidth);
                end
            end
         
            
            % Places Orange Circle onto the image

            if ~isempty(uv_centroidsOrange)

                for i = 1:size(uv_centroidsOrange, 1)
                coordsOrange = [uv_centroidsOrange(i, 1), uv_centroidsOrange(i, 2)];
                propsOrange = regionprops(checkColorOrange, 'MajorAxisLength', 'MinorAxisLength');
                majorAxisOrange = propsOrange(1).MajorAxisLength;
                minorAxisOrange = propsOrange(1).MinorAxisLength;
    
                %diameterOrange = mean([majorAxisOrange, minorAxisOrange]);
                %radiusOrange = diameterOrange / 2;
                radiusOrange = 20;
    
        
                positionOrange = [coordsOrange(1), coordsOrange(2), radiusOrange];
                shapeColorOrange = 'white';
    
                %colors = [colors, {"Orange"}];
                imgWCircles = insertShape(imgWCircles, shapeType, positionOrange, 'Color', shapeColorOrange, 'Linewidth', lineWidth);
                end
            end


            % Places Red Circle onto the image

            if ~isempty(uv_centroidsRed)

                for i = 1:size(uv_centroidsRed, 1)
                coordsRed = [uv_centroidsRed(i, 1), uv_centroidsRed(i, 2)];
                propsRed = regionprops(checkColorRed, 'MajorAxisLength', 'MinorAxisLength');
                majorAxisRed = propsRed(1).MajorAxisLength;
                minorAxisRed = propsRed(1).MinorAxisLength;
    
                %diameterRed = mean([majorAxisRed, minorAxisRed]);
                %radiusRed = diameterRed / 2;
                radiusRed = 20;
    
    
                positionRed = [coordsRed(1), coordsRed(2), radiusRed];
                shapeColorRed = 'red';
    
                %colors = [colors, {"Red"}];
                imgWCircles = insertShape(imgWCircles, shapeType, positionRed, 'Color', shapeColorRed, 'Linewidth', lineWidth);
                end
            end


            % Places Yellow Circle onto the image

            if ~isempty(uv_centroidsYellow)
                for i = 1:size(uv_centroidsYellow, 1)
                coordsYellow = [uv_centroidsYellow(i, 1), uv_centroidsYellow(i, 2)];
                propsYellow = regionprops(checkColorYellow, 'MajorAxisLength', 'MinorAxisLength');
                majorAxisYellow = propsYellow(1).MajorAxisLength;
                minorAxisYellow = propsYellow(1).MinorAxisLength;
    
                %diameterYellow = mean([majorAxisYellow, minorAxisYellow]);
                %radiusYellow = diameterYellow / 2;
                radiusYellow = 20;
    
    
                positionYellow = [coordsYellow(1), coordsYellow(2), radiusYellow];
                shapeColorYellow = 'yellow';
    
                %colors = [colors, {"Yellow"}];
                imgWCircles = insertShape(imgWCircles, shapeType, positionYellow, 'Color', shapeColorYellow, 'Linewidth', lineWidth);
                end
            end



            figure(5);
            imshow(imgWCircles);




            end
            




            % Hint: bwlabel will be very helpful in this function
            % https://www.mathworks.com/help/images/ref/bwlabel.html

            % Hint: regionprops will also be very helpful
            % https://www.mathworks.com/help/images/ref/regionprops.html



        end

        function ts_centroids = correct_centroids(self, centroids, nvargs)
            arguments
                self ImageProcessor;
                centroids double;
                nvargs.ball_z = 10;  % in mm. This is the height of the centroid
            end
            %CORRECT_CENTROIDS transforms image coordinate centroids into
            %task-space coordinates for the ball
            % Inputs: 
            %   centroids: a [nx2] array of centroids in image coordinates
            %   ball_z (optional): how high the center of the ball is in
            %                      millimeters

            % YOUR CODE HERE9
            
            %initializing output
            ts_centroids = zeros(size(centroids));

            %measured distance from (0,0) to base of camera in checkerboard coords
            cam_ckb = [115,280]; %mm

            %for every centroid,
            for i = 1:height(centroids)
                %convert to checkerboard
                cent_ckb = pointsToWorld(self.camera.getCameraIntrinsics() , ...
                                             self.camera.getRotationMatrix()   , ...
                                             self.camera.getTranslationVector(), ...
                                             centroids(i,:));     

            %Length from PTW coords to cam
            L = sqrt((cent_ckb(1)-cam_ckb(1))^2 + (cent_ckb(2)-cam_ckb(2))^2);
            
            %using similar triangles to find the distance l between PTW and
            %actual coords
            l = 10*L/190;

            %theta is the angle of the triangle from the camera to PTW
            %points
            theta = acos((280-cent_ckb(2))/L);

            %find x and y components of l
            l_x = l*cos(theta);
            l_y = l*sin(theta);

            %find actual coordinates by taking the difference
            a_x = cent_ckb(1) - l_x;
            a_y = cent_ckb(2) - l_y;

            %convert from checker to robot
             ts_centroids(i,1:2) = [105 + a_y, -95  + a_x];

            end

            % Hint: similar triangles

            % Hint2: imageToWorld will be helpful here
        end

        % @@@@@@@@@@@@@@
        % YOUR CODE HERE
        % @@@@@@@@@@@@@@
        % Write a function that acquires an image, returns the
        % coordinates of the balls in the task space of the robot. Satisfy
        % the following requiremetns.
        
        %DETECT_BALLS finds the task space coordinates of all balls on the
        %checkerboard
        % Outputs:
        %   ts_coords: the task space coordinates of all balls in the
        %              workspace

    function [ts_coords, colorsOnTable] = detect_balls(self)
        Vlogging = self.camera;
        
        bw = self.generate_static_mask(); % Create the mask the outlines the checkerboard
        
        frame = Vlogging.getImage(); % Take picture of board
        % figure(2);
        % imshow(frame);
        
        [greenBW, ~] = greenMaskFINAL(frame);  % Mask only green balls
        [orangeBW, ~] = orangeMaskFINAL(frame); % Mask only orange balls
        [redBW, ~] = redMaskFINAL(frame);       % Mask only red balls
        [yellowBW, ~] = yellowMaskFINAL2(frame); % Mask only yellow balls
        
        combinedMask = greenBW | orangeBW | redBW | yellowBW; % Combine each color
        imageDimensions = size(combinedMask);
        finalMask = bw & combinedMask; % Create the final mask by combining the static mask and color masks
        
        % Combine each color mask and the checkerboard mask
        orangeMask = bw & orangeBW;
        yellowMask = bw & yellowBW;
        greenMask = bw & greenBW;
        redMask = bw & redBW;
        
        %figure(3) % Display combined color mask
        %imshow(greenMask);
        
        % figure(4) % Display framed color mask
        % imshow(finalMask);
        
      
        [colors, uv_centroids] = self.detect_centroids(bw); % Give detect centroid the checkerboard mask
        colorsOnTable = [];

        colorsOnTable = colors;


        ts_coords = zeros(size(uv_centroids));

        
        ts_coords = self.correct_centroids(uv_centroids);


         % for i = 1:height(ts_coords)
         %     ts_coords(i, :) = self.image_to_robot(ts_coords(i, :));
         % end
    end
    end
end