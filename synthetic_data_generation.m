% Clear workspace
clear; clc; close all;

% Define camera intrinsic parameters
focal_length = [608.210845, 608.210845];  % Focal lengths (fx, fy)
principal_point = [640 440];  % Principal point (cx, cy)
image_size = [880 1280];  % Image dimensions (rows, cols)
camera_matrix = [focal_length(1), 0, principal_point(1);
                 0, focal_length(2), principal_point(2);
                 0, 0, 1];

% Define circle parameters in 3D
circle1_center = [10.9349; -10.1; 5.2508];  % (x, y, z) position in meters
circle1_radius = 0.5;        % Radius in meters
circle1_normal = [0, 1, 0];  % Normal vector

circle2_center = [10.9349; -10.1; 5.0];  % (x, y, z) position in meters
circle2_radius = 0.1;        % Radius in meters
circle2_normal = [0, 1, 0];  % Normal vector (not aligned with axes)

% Normalize normal vectors
circle1_normal = circle1_normal / norm(circle1_normal);
circle2_normal = circle2_normal / norm(circle2_normal);

% Define camera transformation matrix (4x4)
% 10.8951687815517904 -8.8267950003749771 4.4964996589787587 0.7474290581078724 -0.0045493735392005 -0.0490547097888047 0.6625124464822644
camera_translation = [10.8951687815517904 -8.8267950003749771 4.4964996589787587];  % Camera position in world coordinates
camera_quaternion = [0.6625124464822644 0.7474290581078724 -0.0045493735392005 -0.0490547097888047];
camera_rotation = quat2rotm(camera_quaternion);        % Camera aligned with world axes (Z-forward, X-right, Y-down)
camera_transformation = [camera_rotation, camera_translation';
                         0, 0, 0, 1];  % Full 4x4 transformation matrix

% Helper function to generate points on a circle in 3D
generate_circle_points = @(center, radius, normal, num_points) ...
    rotate_circle_to_normal(center, radius, normal, num_points);

% Generate 3D points for circles
num_points = 100;
circle1_points_3D = generate_circle_points(circle1_center, circle1_radius, circle1_normal, num_points);
circle2_points_3D = generate_circle_points(circle2_center, circle2_radius, circle2_normal, num_points);

%여기서부터 내 원래 코드를 기반으로 리프로젝션 되도록 만들기
% % Transform points into camera coordinate frame
% world_to_camera = inv(camera_transformation);  % Inverse of transformation matrix
% circle1_camera_frame = world_to_camera(1:3, 1:3) * (circle1_points_3D - camera_translation') + world_to_camera(1:3, 4);
% circle2_camera_frame = world_to_camera(1:3, 1:3) * (circle2_points_3D - camera_translation') + world_to_camera(1:3, 4);
% P1_camera_homogeneous = camera_rotation * circle1_points_3D + camera_translation';
% P2_camera_homogeneous = camera_rotation * circle2_points_3D + camera_translation';
P1_camera_homogeneous = camera_rotation * (circle1_points_3D - camera_translation');
P2_camera_homogeneous = camera_rotation * (circle2_points_3D - camera_translation');

% % Project 3D points to 2D image plane
% project_to_2D = @(points_3D) camera_matrix * points_3D ./ points_3D(3, :);
% circle1_points_2D = project_to_2D(circle1_camera_frame);
% circle2_points_2D = project_to_2D(circle2_camera_frame);
circle1_points_2D = camera_matrix * P1_camera_homogeneous;
circle2_points_2D = camera_matrix * P2_camera_homogeneous;

% Convert from homogeneous to 2D coordinates
circle1_points_2D(1, :) = circle1_points_2D(1, :) ./ circle1_points_2D(3, :);
circle1_points_2D(2, :) = circle1_points_2D(2, :) ./ circle1_points_2D(3, :);

circle2_points_2D(1, :) = circle2_points_2D(1, :) ./ circle2_points_2D(3, :);
circle2_points_2D(2, :) = circle2_points_2D(2, :) ./ circle2_points_2D(3, :);


% Extract 2D points
circle1_points_2D = circle1_points_2D(1:2, :)';
circle2_points_2D = circle2_points_2D(1:2, :)';

% Visualize in 3D space
figure; hold on; axis equal;
grid on; view(45, 45);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Visualization of Circles and Virtual Camera');

% Plot the circles in 3D
plot3(circle1_points_3D(1, :), circle1_points_3D(2, :), circle1_points_3D(3, :), 'r-', 'LineWidth', 1.5);
plot3(circle2_points_3D(1, :), circle2_points_3D(2, :), circle2_points_3D(3, :), 'b-', 'LineWidth', 1.5);

% Plot the circle centers
plot3(circle1_center(1), circle1_center(2), circle1_center(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(circle2_center(1), circle2_center(2), circle2_center(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Draw the normal vectors
quiver3(circle1_center(1), circle1_center(2), circle1_center(3), ...
        circle1_normal(1), circle1_normal(2), circle1_normal(3), ...
        0.5, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver3(circle2_center(1), circle2_center(2), circle2_center(3), ...
        circle2_normal(1), circle2_normal(2), circle2_normal(3), ...
        0.5, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Plot the camera
plot_camera_frame(camera_rotation, camera_translation', 0.5, 'r');
legend('Circle 1', 'Circle 2', 'Circle 1 Center', 'Circle 2 Center', 'Circle 1 Normal', 'Circle 2 Normal', ...
       'Camera Position', 'Camera Orientation');
hold off;
% f = FigureRotator();
% plot_inertial_frame(3);

% Visualize the 2D projection
figure; hold on; axis equal;
xlim([0, image_size(2)]); ylim([0, image_size(1)]);
set(gca, 'YDir', 'reverse');  % Invert the y-axis
title('Synthetic Image with Reprojected Circles');
xlabel('X-axis (pixels)'); ylabel('Y-axis (pixels)');
plot(circle1_points_2D(:, 1), circle1_points_2D(:, 2), 'r-', 'LineWidth', 1.5);
plot(circle2_points_2D(:, 1), circle2_points_2D(:, 2), 'b-', 'LineWidth', 1.5);

disp('3D Visualization and 2D projection completed.');

% Helper function to align circle with normal vector
function points_3D = rotate_circle_to_normal(center, radius, normal, num_points)
    % Generate default circle in XY plane
    theta = linspace(0, 2*pi, num_points);
    circle_points = [radius * cos(theta);
                     radius * sin(theta);
                     zeros(1, num_points)];
                 
    % Find rotation matrix to align Z-axis to normal vector
    normal = normal / norm(normal);
    v = cross([0, 0, 1], normal);
    s = norm(v);
    c = dot([0, 0, 1], normal);
    if s == 0
        R = eye(3);  % Aligned with Z-axis already
    else
        vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];  % Skew-symmetric cross-product matrix
        R = eye(3) + vx + vx^2 * (1 - c) / s^2;
    end
    
    % Rotate circle and translate to center
    points_3D = R * circle_points + center;
end
