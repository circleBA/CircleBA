%% Draw Ellipse on Image
function figure_detected_two_ellipse(imagePath, ellipses_params, tstampStr)
    % Read the image
    img = imread(imagePath);
    [img_height, img_width, ~] = size(img);

    imshow(img);
    hold on;

    axis on;
    axis image;             % ensures correct scaling
    set(gca,'YDir','reverse');  % maintain top-left as (0,0)

    for i = 1:2
        k = ellipses_params(i, 1); %y
        h = ellipses_params(i, 2); %x
        a = ellipses_params(i, 3)/2; %a
        b = ellipses_params(i, 4)/2; %b
        angle = ellipses_params(i, 5); %angle (deg)
    
        theta = deg2rad(angle);
        t = linspace(0, 2*pi, 100);
        X = a * cos(t);
        Y = b * sin(t);
        
        ellipse_orientation = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        ellipse_points = ellipse_orientation * [X; Y];
        
        ellipse_points(1, :) = ellipse_points(1, :) + h;
        ellipse_points(2, :) = ellipse_points(2, :) + k;
    
        % Calculate and plot the major axis
        major_axis_points = ellipse_orientation * [a, -a; 0, 0];
        major_axis_points(1, :) = major_axis_points(1, :) + h;
        major_axis_points(2, :) = major_axis_points(2, :) + k;
        
        % Calculate and plot the minor axis
        minor_axis_points = ellipse_orientation * [0, 0; b, -b];
        minor_axis_points(1, :) = minor_axis_points(1, :) + h;
        minor_axis_points(2, :) = minor_axis_points(2, :) + k;
        
         % figure;
        
        
        % Plot the ellipse
        if i == 1
            plot(ellipse_points(1, :), ellipse_points(2, :), 'b', 'LineWidth', 4); 
        else
            plot(ellipse_points(1, :), ellipse_points(2, :), 'r', 'LineWidth', 4);
        end
        % plot(major_axis_points(1, :), major_axis_points(2, :), 'r--', 'LineWidth', 1.5);
        % plot(minor_axis_points(1, :), minor_axis_points(2, :), 'g--', 'LineWidth', 1.5); hold on;
        xlim([0, 1280]);
        ylim([0, 880]);
    end
    % hold off;
    % Set plot title and labels
    title('Detected Ellipse');
    text(15, 15, sprintf('#%s ', tstampStr), 'FontSize', 15, 'color', 'r'); hold on; grid on; axis equal
    xlabel('X-axis');
    ylabel('Y-axis');

end