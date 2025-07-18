clc;
close all;
clear variables; %clear classes;

%%
load('data/circle56_p2c.mat');
next = load('data/circle61_p2c.mat');
sample_num = 1000;

%% Cone Equation
% Visualize the cone
figure;
plot_inertial_frame(1); hold on; % Camera Frame
for i = 1:2
    k = ellipses_params(i, 1); %y
    h = ellipses_params(i, 2); %x
    a = ellipses_params(i, 3)/2; %a
    b = ellipses_params(i, 4)/2; %b
    theta = 90-ellipses_params(i, 5); %angle (deg)
    t = linspace(0, 2*pi, sample_num);
    x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
    y_ellipse = k + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
    x_cam = (x_ellipse - intrinsics.K(1, 3)) / intrinsics.K(1, 1);
    y_cam = (y_ellipse - intrinsics.K(2, 3)) / intrinsics.K(2, 2);
    % Define the rays (from camera center through each ellipse point)
    num_points = length(x_cam);
    rays = zeros(3, num_points);
    scaling_factor = 10;
    for j = 1:num_points
        % Each ray in normalized camera coordinates
        rays(:, j) = [x_cam(j); y_cam(j); 1];
        % Normalize the ray
        rays(:, j) = rays(:, j) / norm(rays(:, j));
        rays(:, j) = scaling_factor * rays(:, j);
    end
    % Plot the rays
    for j = 1:40:num_points
        plot3([0 rays(1, j)], ...
              [0 rays(2, j)], ...
              [0 rays(3, j)], 'g-');
    end
    % Plot the ellipse points on the image plane (z = 1)
    plot3(x_cam, y_cam, ones(1, num_points), 'mo');
end
%PM circle
if abs(PM_normal_cam(1, 1)) < abs(PM_normal_cam(2, 1)) && abs(PM_normal_cam(1, 1)) < abs(PM_normal_cam(3, 1))
    v = [1, 0, 0];
elseif abs(PM_normal_cam(2, 1)) < abs(PM_normal_cam(1, 1)) && abs(PM_normal_cam(2, 1)) < abs(PM_normal_cam(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(PM_normal_cam(:, 1), v);
u = u / norm(u);
v = cross(PM_normal_cam(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
plot3(PM_center_cam(1, 1) + PM_circle_3D(1, :), PM_center_cam(2, 1) + PM_circle_3D(2, :), PM_center_cam(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;

%Window circle
if abs(Window_normal_cam(1, 1)) < abs(Window_normal_cam(2, 1)) && abs(Window_normal_cam(1, 1)) < abs(Window_normal_cam(3, 1))
    v = [1, 0, 0];
elseif abs(Window_normal_cam(2, 1)) < abs(Window_normal_cam(1, 1)) && abs(Window_normal_cam(2, 1)) < abs(Window_normal_cam(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(Window_normal_cam(:, 1), v);
u = u / norm(u);
v = cross(Window_normal_cam(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
plot3(Window_center_cam(1, 1) + Window_circle_3D(1, :), Window_center_cam(2, 1) + Window_circle_3D(2, :), Window_center_cam(3, 1) + Window_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;


axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
ylim([-10 10]);
zlim([-10 10]);
xlim([-10 10]);
title('3D Visualization of Cone in Camera Frame');
hold off;
f = FigureRotator();




%% Reproject test with true pose
InitialMap_PM = [PM_center_cam(1, 1) + PM_circle_3D(1, :); PM_center_cam(2, 1) + PM_circle_3D(2, :); PM_center_cam(3, 1) + PM_circle_3D(3, :)];
InitialMap_Win = [Window_center_cam(1, 1) + Window_circle_3D(1, :); Window_center_cam(2, 1) + Window_circle_3D(2, :); Window_center_cam(3, 1) + Window_circle_3D(3, :)];



% T1_inv = inv(T_iss_cam);
% T_relative = T1_inv * next.T_iss_cam;
% T_relative = inv(T_relative);
% Rk = T_relative(1:3, 1:3);
% Tk = T_relative(1:3, 4);
% 
% for i=1:2
%     figure_detected_ellipse(next.imagePath, next.ellipses_params(i,:));
%     figure_detected_ellipse(imagePath, ellipses_params(i,:));
%     % 1st map
%     % Transform 3D points to the camera coordinate system of the next frame
%     for j = 1:sample_num
%         P_camera = Rk * InitialMap_PM(:, j) + Tk;
% 
%         % Convert to homogeneous coordinates by appending a row of ones
%         P_camera_homogeneous = P_camera;
% 
%         % Project 3D points to 2D using intrinsic matrix
%         projected_points = intrinsics.K * P_camera_homogeneous;
% 
%         % Convert from homogeneous to 2D coordinates
%         projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
%         projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
% 
%         % Extract the 2D points
%         u1(j) = projected_points(1, :);
%         v1(j) = projected_points(2, :);
% 
%         %reprojection error of ellipse constraint
%         e1(j) = abs([u1(j) v1(j) 1]* next.C_quad{i} * [u1(j); v1(j); 1]);
%     end
%     %second map
%     for j = 1:sample_num
%         P_camera = Rk * InitialMap_Win(:, j) + Tk;
% 
%         % Convert to homogeneous coordinates by appending a row of ones
%         P_camera_homogeneous = P_camera;
% 
%         % Project 3D points to 2D using intrinsic matrix
%         projected_points = intrinsics.K * P_camera_homogeneous;
% 
%         % Convert from homogeneous to 2D coordinates
%         projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
%         projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
% 
%         % Extract the 2D points
%         u2(j) = projected_points(1, :);
%         v2(j) = projected_points(2, :);
% 
%         e2(j) = abs([u2(j) v2(j) 1]* next.C_quad{i} * [u2(j); v2(j); 1]);
%     end
% end

% Visualization

img = imread(next.imagePath);
figure;
imshow(img); 
hold on;
for i = 1:2
    plot(u1, v1, 'bo', 'MarkerSize',1); hold on;% Plot reprojected points in red
    plot(u2, v2, 'ro', 'MarkerSize',1); % Plot reprojected points in red
end
legend('First Circle', 'Second Circle');
title('Reprojected 2D Points on Image Plane');
hold off;


%% Optimization
Qpm = next.C_quad{1};
Qwin = next.C_quad{2};
T_last_inv = inv(T_iss_cam);
T_relative = T_last_inv * next.T_iss_cam;
T_relative = inv(T_relative);
R_true = T_relative(1:3, 1:3);
T_true = T_relative(1:3, 4);
R_init = eye(3);
T_init = [0;0;0];
% R_init = R_true*[0.99 0 0; 0 0.99 0; 0 0 0.99];
% T_init = T_true+[0.1; -0.1; -0.1];

%optimization per frame
objective_function = @(params) EReprojection(params, InitialMap_PM, InitialMap_Win, Qpm, Qwin, intrinsics.K);
initial_params = [R_init, T_init];

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
[opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);

R_relative_opt= opt_params(1:3, 1:3);
T_relative_opt = opt_params(1:3, 4);
Relative_Trans_opt = [R_relative_opt, T_relative_opt; 0 0 0 1];

EstimatedPose = T_iss_cam * Relative_Trans_opt;

%visualize reprojection with optimized R,t
for i = 1:sample_num
    P_camera = R_relative_opt * InitialMap_Win(:, i) + T_relative_opt;
    
    % Convert to homogeneous coordinates by appending a row of ones
    P_camera_homogeneous = P_camera;
    
    % Project 3D points to 2D using intrinsic matrix
    projected_points = intrinsics.K * P_camera_homogeneous;
    
    % Convert from homogeneous to 2D coordinates
    projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    
    % Extract the 2D points
    u1(i) = projected_points(1, :);
    v1(i) = projected_points(2, :);

    e1(i) = abs([u1(i) v1(i) 1]* Qpm * [u1(i); v1(i); 1]);
end

for i = 1:sample_num
    P_camera = R_relative_opt * InitialMap_PM(:, i) + T_relative_opt;
    
    % Convert to homogeneous coordinates by appending a row of ones
    P_camera_homogeneous = P_camera;
    
    % Project 3D points to 2D using intrinsic matrix
    projected_points = intrinsics.K * P_camera_homogeneous;
    
    % Convert from homogeneous to 2D coordinates
    projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    
    % Extract the 2D points
    u2(i) = projected_points(1, :);
    v2(i) = projected_points(2, :);

    e2(i) = abs([u2(i) v2(i) 1]* Qwin * [u2(i); v2(i); 1]);
end

% Visualization

img = imread(next.imagePath);
figure;
imshow(img); 
hold on;
plot(u1, v1, 'mo', 'MarkerSize',1); hold on;% Plot reprojected points in red
plot(u2, v2, 'co', 'MarkerSize',1); % Plot reprojected points in red
legend('First Circle', 'Second Circle');
title('Reprojected 2D Points on Image Plane with optimized R T');
hold off;


%%
function F = EReprojection(params, PM_Point3D, Win_Point3D, Qpm, Qwin, K)
    Rk = params(1:3, 1:3);
    Tk = params(1:3, 4);
    n = size(PM_Point3D, 2);
    F1 = zeros(n, 1);
    F2 = zeros(n, 1);
    for i = 1:n
        % 3D point
        pm_point3D = PM_Point3D(:, i);
        win_point3D = Win_Point3D(:, i);

        pm_projected_point = K * (Rk * pm_point3D + Tk);
        pm_projected_point = pm_projected_point(1:2) / pm_projected_point(3);
        pm_error = [pm_projected_point; 1]' * Qpm * [pm_projected_point; 1];

        win_projected_point = K * (Rk * win_point3D + Tk);
        win_projected_point = win_projected_point(1:2) / win_projected_point(3);
        win_error = [win_projected_point; 1]' * Qwin * [win_projected_point; 1];

        F1(i) = pm_error;
        F2(i) = win_error;
    end
    F = sum(F1) + sum(F2);
end

