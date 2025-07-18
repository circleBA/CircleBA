% Clear workspace

clear; clc; close all;
% posePath = 'data/td_yaw/groundtruth_selected.txt';
% imagePath = 'data/td_yaw/gray_selected/';

%posePath = 'data/ff_return_journey_forward/groundtruth_selected.txt';
% imagePath = 'data/ff_return_journey_forward/gray_selected/';
posePath = 'data/multiview_mixed_data/groundtruth.txt';
% imagePath = 'data/synthetic/';
% 원래 이걸로되어있음
% posePath = 'data/multiview_mixed_data/groundtruth.txt';
% imagePath = 'data/multiview_mixed_data/gray/';

% posePath = 'data/multiview_mixed_data_syn/groundtruth.txt';
% imagePath = 'data/multiview_mixed_data_syn/gray/';

%% ToDO
% 트래킹 넘을 2장 이상으로 해야하고 -  완
% 내 생각에 문제는 synthetic에서조차 initial map을 시각화했을때 두 원이 같은 평면상에 있지 않음..! - 해결함
% T_p2c_cam_true{2} 가 혼자 y값이 이상함 - 완

%% Step1: Generate Synthetic Circle Data

% Define camera intrinsic parameters
focal_length = [608.210845, 608.210845];  % Focal lengths (fx, fy)
principal_point = [640 440];  % Principal point (cx, cy)
image_size = [880 1280];  % Image dimensions (rows, cols)
camera_matrix = [focal_length(1), 0, principal_point(1);
                 0, focal_length(2), principal_point(2);
                 0, 0, 1];

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
R = [0.5, 0.1];
% Define circle parameters in 3D
circle1_center = [10.9349; -10.1; 5.2508];  % (x, y, z) position in meters
circle1_radius = R(1);        % Radius in meters
circle1_normal = [0, 1, 0];  % Normal vector

circle2_center = [10.9349; -10.1; 5.0];  % (x, y, z) position in meters
circle2_radius = R(2);        % Radius in meters
circle2_normal = [0, 1, 0];  % Normal vector (not aligned with axes)

% Normalize normal vectors
circle1_normal = circle1_normal / norm(circle1_normal);
circle2_normal = circle2_normal / norm(circle2_normal);

% Define camera transformation matrix (4x4) // Ground Truth Datas

true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(true_pose,1)
    tstamp{i} = true_pose(i, 1);
    P_iss_cam_true(i, :) = true_pose(i, :);
    R_iss_cam_true{i} = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]); 
    t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)'; % camera_translation
    T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
end

tracking_num = size(true_pose,1);
sample_num = 1000;

% Helper function to generate points on a circle in 3D
generate_circle_points = @(center, radius, normal, num_points) ...
    rotate_circle_to_normal(center, radius, normal, num_points);

% Generate 3D points for circles
num_points = 1000;
circle1_points_3D = generate_circle_points(circle1_center, circle1_radius, circle1_normal, num_points);
circle2_points_3D = generate_circle_points(circle2_center, circle2_radius, circle2_normal, num_points);

% % Transform points into camera coordinate frame
for i=1:tracking_num
    circle1_points_2D{i} = project_3D_to_2D(circle1_points_3D', intrinsics.K, T_iss_cam_true{i});
    circle2_points_2D{i} = project_3D_to_2D(circle2_points_3D', intrinsics.K, T_iss_cam_true{i});
end

    % for i = 1:tracking_num
%     P1_camera_homogeneous{i} = R_iss_cam_true{i} * (circle1_points_3D - t_iss_cam_true{i});
%     P2_camera_homogeneous{i} = R_iss_cam_true{i} * (circle2_points_3D - t_iss_cam_true{i});
% end
% for i = 1:tracking_num
%     projected_1 = intrinsics.K * P1_camera_homogeneous{i};
%     projected_2 = intrinsics.K * P2_camera_homogeneous{i};
% 
%     % Normalize homogeneous coordinates (divide by depth Z)
%     circle1_points_2D{i} = projected_1(1:2, :) ./ projected_1(3, :);
%     circle2_points_2D{i} = projected_2(1:2, :) ./ projected_2(3, :);
% end
%%
% % MINJI 0407_1 start
% figure; hold on; axis equal;
% grid on; view(45, 45);
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('3D Visualization of Circles and Virtual Camera');
% % Plot the circles in 3D
% plot3(circle1_points_3D(1, :), circle1_points_3D(2, :), circle1_points_3D(3, :), 'r-', 'LineWidth', 1.5); hold on;
% plot3(circle2_points_3D(1, :), circle2_points_3D(2, :), circle2_points_3D(3, :), 'b-', 'LineWidth', 1.5);
% 
% % Plot the circle centers
% plot3(circle1_center(1), circle1_center(2), circle1_center(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% plot3(circle2_center(1), circle2_center(2), circle2_center(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% % Draw the normal vectors
% quiver3(circle1_center(1), circle1_center(2), circle1_center(3), ...
%         circle1_normal(1), circle1_normal(2), circle1_normal(3), ...
%         0.5, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
% quiver3(circle2_center(1), circle2_center(2), circle2_center(3), ...
%         circle2_normal(1), circle2_normal(2), circle2_normal(3), ...
%         0.5, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
% 
% % Plot the camera
% for i=1:tracking_num
%     plot_camera_frame(R_iss_cam_true{i}, t_iss_cam_true{i}, 0.5, 'r'); hold on;
%     % pause(0.5);
% end
% legend('Circle 1', 'Circle 2', 'Circle 1 Center', 'Circle 2 Center', 'Circle 1 Normal', 'Circle 2 Normal', ...
%        'Camera Position', 'Camera Orientation');
% hold off;
% % MINJI 0407_1 end
%% Step 2: Circle BA

%% Step 2-1: Extract Ellipsoidal Expression
%VIsualize
% Visualize the 2D projection
figure; hold on; axis equal;
xlim([0, image_size(2)]); ylim([0, image_size(1)]);
set(gca, 'YDir', 'reverse');  % Invert the y-axis
title('Synthetic Image with Reprojected Circles');
xlabel('X-axis (pixels)'); ylabel('Y-axis (pixels)');
for i = 1:tracking_num
    plot(circle1_points_2D{i}(:, 1), circle1_points_2D{i}(:, 2), 'r-', 'LineWidth', 1.5); hold on;
    plot(circle2_points_2D{i}(:, 1), circle2_points_2D{i}(:, 2), 'b-', 'LineWidth', 1.5);
    % pause(0.5);
end


% Visualize
% Compute ellipse parameters for both circles
for i = 1:tracking_num
    ellipse_result1{i}(1, :) = fit_ellipse(circle1_points_2D{i});
    ellipse_result2{i}(1, :) = fit_ellipse(circle2_points_2D{i});
end

% Display results
% disp('Ellipse 1 Parameters (x_center, y_center, major_axis, minor_axis, theta):');
% disp(ellipse1);
% 
% disp('Ellipse 2 Parameters (x_center, y_center, major_axis, minor_axis, theta):');
% disp(ellipse2);
%image = './data/ff_return_journey_forward/gray/1617290600939388160.png';

PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);

for i = 1:tracking_num
    ellipse_params1 = ellipse_result1{i};
    ellipse_params2 = ellipse_result2{i};
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    %plot(ellipse_params1{1}, ellipse_params1{2}, 'yo', 'MarkerSize', 10, 'LineWidth', 2);
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);

    % Visualize Detected Ellipse
    % figure;
    % figure_detected_two_ellipse(image, ellipses_params{i}, 'data/synthetic/1563961557613548032.png'); hold on;
    % pause(0.5);


    % % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 20
        ransac(i) = 0;
    % else
    %     if i == 1 
    %         tstampStr = sprintf('%.0f', tstamp(i));
    %         image = strcat(imagePath, imgList(i, :));
    %         figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
    %         pause(4);
    %     end
    %     tstampStr = sprintf('%.0f', tstamp(i));
    %     image = strcat(imagePath, imgList(i, :));
    %     figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
    %     pause(0.5);
    end

    z = ( PM_center_cam{i} - Window_center_cam{i} ) / norm(PM_center_cam{i} - Window_center_cam{i});
    x = cross(PM_normal_cam{i}, z);
    x = x/norm(x);
    z = cross(x, PM_normal_cam{i});
    y = PM_normal_cam{i};
    R_cam_p2c = [x, y, z];
    t_cam_p2c = PM_center_cam{i};
    T_cam_p2c{i} = [R_cam_p2c, t_cam_p2c; 0 0 0 1];
    T_p2c_cam{i} = T_cam_p2c{1} * inv(T_cam_p2c{i});
    
end
ransac = logical(ransac);
tstamp = tstamp(ransac);
% imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
T_p2c_cam = T_p2c_cam(ransac);
T_cam_p2c = T_cam_p2c(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);

tracking_num = size(R_iss_cam_true, 2);

% tracking_num = size(R_iss_cam_true, 2);
camera_translation = [10.8951687815517904 -7.8267950003749771 3.4964996589787587];  % Camera position in world coordinates
camera_quaternion = [0.6625124464822644 0.7474290581078724 -0.0045493735392005 -0.0490547097888047];
camera_rotation = quat2rotm(camera_quaternion);        % Camera aligned with world axes (Z-forward, X-right, Y-down)
camera_transformation = [camera_rotation, camera_translation';
                         0, 0, 0, 1];  % Full 4x4 transformation matrix


T_p2c_iss = T_p2c_cam{1} * inv(T_iss_cam_true{1});
for i = 1:tracking_num
    T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
end
% T_p2c_iss = T_p2c_cam{1} * inv(T_iss_cam_true{1});
% for i = 1:tracking_num
%     T_p2c_iss = T_p2c_cam{i} * inv(T_iss_cam_true{i});
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end

%% Step 2-2: Choose Initial Map Points with Cone Equation
sample_num=1000;
figure;
plot_inertial_frame(1); hold on; % Camera Frame
for i = 1:2
    k = ellipses_params{1}(i, 1); %y
    h = ellipses_params{1}(i, 2); %x
    a = ellipses_params{1}(i, 3)/2; %a
    b = ellipses_params{1}(i, 4)/2; %b
    theta = 90-ellipses_params{1}(i, 5); %angle (deg)
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
if abs(PM_normal_cam{1}(1, 1)) < abs(PM_normal_cam{1}(2, 1)) && abs(PM_normal_cam{1}(1, 1)) < abs(PM_normal_cam{1}(3, 1))
    v = [1, 0, 0];
elseif abs(PM_normal_cam{1}(2, 1)) < abs(PM_normal_cam{1}(1, 1)) && abs(PM_normal_cam{1}(2, 1)) < abs(PM_normal_cam{1}(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(PM_normal_cam{1}(:, 1), v);
u = u / norm(u);
v = cross(PM_normal_cam{1}(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
plot3(PM_center_cam{1}(1, 1) + PM_circle_3D(1, :), PM_center_cam{1}(2, 1) + PM_circle_3D(2, :), PM_center_cam{1}(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;

%Window circle
if abs(Window_normal_cam{1}(1, 1)) < abs(Window_normal_cam{1}(2, 1)) && abs(Window_normal_cam{1}(1, 1)) < abs(Window_normal_cam{1}(3, 1))
    v = [1, 0, 0];
elseif abs(Window_normal_cam{1}(2, 1)) < abs(Window_normal_cam{1}(1, 1)) && abs(Window_normal_cam{1}(2, 1)) < abs(Window_normal_cam{1}(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(Window_normal_cam{1}(:, 1), v);
u = u / norm(u);
v = cross(Window_normal_cam{1}(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
plot3(Window_center_cam{1}(1, 1) + Window_circle_3D(1, :), Window_center_cam{1}(2, 1) + Window_circle_3D(2, :), Window_center_cam{1}(3, 1) + Window_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;


InitialMap_PM = [PM_center_cam{1}(1, 1) + PM_circle_3D(1, :); PM_center_cam{1}(2, 1) + PM_circle_3D(2, :); PM_center_cam{1}(3, 1) + PM_circle_3D(3, :)];
InitialMap_Win = [Window_center_cam{1}(1, 1) + Window_circle_3D(1, :); Window_center_cam{1}(2, 1) + Window_circle_3D(2, :); Window_center_cam{1}(3, 1) + Window_circle_3D(3, :)];

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

%% Step 2-3: Tracking Optimization
disp("optimization");
ransac2 = ones(1, tracking_num);
EstimatedPose{1} = T_p2c_cam{1};
PM_normal_i = EstimatedPose{1} * [PM_normal_cam{1}; 1];
PM_normal_init = PM_normal_i(1:3)/norm(PM_normal_i(1:3));
figure;
for i = 2:tracking_num
    T_relative = inv( inv(T_p2c_cam{1}) * T_p2c_cam{i} ); % 원래 초기값은 P2C
    % T_relative = eye(4);
    [yaw, pitch, roll] = dcm2angle(T_relative(1:3, 1:3));
    % stateEsti_init = [roll, pitch, yaw, T_relative(1, 4), T_relative(2, 4), T_relative(3, 4)]; % roll, pitch, yaw, x, y, z
    xi_init = logSE3(T_relative);
    

    %optimization per frame
    % objective_function = @(params) EReprojection_motiononly(params, InitialMap_PM, InitialMap_Win, C_pm{i}, C_win{i}, intrinsics.K);
    objective_function = @(params) EReprojection_motiononly3( ...
    params, ...
    InitialMap_PM, ...
    InitialMap_Win, ...
    C_pm{i}, ...
    C_win{i}, ...
    ellipses_params{i}, ...
    PM_center_cam(1), ...
    intrinsics.K);
    initial_params = xi_init;
    
    % options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
    % [opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
                       'Display','iter','FiniteDifferenceType','forward');

    [opt_params, resnorm] = lsqnonlin(objective_function, ...
                                  initial_params, [], [], options);
    disp(opt_params);
    disp(resnorm);
    global final_T;
    Relative_Trans_opt = final_T;

    % opt_params가 6vector se(3)이니까 4by4 SE(3)로 바꿔야함

    % rpy_relative_opt = opt_params(1:3);
    % R_relative_opt = angle2dcm( yaw, pitch, roll );
    % T_relative_opt = opt_params(4:6)';
    % Relative_Trans_opt = [R_relative_opt, T_relative_opt; 0 0 0 1];
    % Relative_Trans_opt = expSE3(opt_params);
    % R_relative_opt = Relative_Trans_opt(1:3, 1:3);
    % T_relative_opt = Relative_Trans_opt(1:3, 4);
    
    EstimatedPose{i} = T_p2c_cam{1} * inv( Relative_Trans_opt );
    % EstimatedPose{i} = T_p2c_cam{i-1} * inv( Relative_Trans_opt );
    
    
    PM_normal = EstimatedPose{i} * [PM_normal_cam{i}; 1];
    PM_normal_esti = PM_normal(1:3) / norm(PM_normal(1:3));
    angle_diff2{i} = acos(dot(PM_normal_init, PM_normal_esti))*180/pi;
    if angle_diff2{i} > 300
        ransac2(i) = 0;
    end
    
    %visualize reprojection with optimized R,t
    % for j = 1:sample_num
    %     P_camera = R_relative_opt * InitialMap_PM(:, j) + T_relative_opt;
    %     % Convert to homogeneous coordinates by appending a row of ones
    %     P_camera_homogeneous = P_camera;
    % 
    %     % Project 3D points to 2D using intrinsic matrix
    %     projected_points = intrinsics.K * P_camera_homogeneous;
    % 
    %     % Convert from homogeneous to 2D coordinates
    %     projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    %     projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    % 
    %     % Extract the 2D points
    %     u1(j) = projected_points(1, :);
    %     v1(j) = projected_points(2, :);
    % 
    %     e1(j) = abs([u1(j) v1(j) 1]* C_pm{i} * [u1(j); v1(j); 1]);
    % end

    for j = 1:sample_num
    % Homogeneous 3D point
        pt3d_h = [InitialMap_PM(:, j); 1];

    % Transform with full SE(3) pose
        pt3d_trans = Relative_Trans_opt * pt3d_h;
    
    % Project with intrinsics
        proj_h = intrinsics.K * pt3d_trans(1:3);
        proj_h = proj_h / proj_h(3);

        u1(j) = proj_h(1);
        v1(j) = proj_h(2);

        e1(j) = abs([u1(j) v1(j) 1] * C_pm{i} * [u1(j); v1(j); 1]);
    end




    % for j = 1:sample_num
    %     P_camera = R_relative_opt * InitialMap_Win(:, j) + T_relative_opt;
    % 
    %     % Convert to homogeneous coordinates by appending a row of ones
    %     P_camera_homogeneous = P_camera;
    % 
    %     % Project 3D points to 2D using intrinsic matrix
    %     projected_points = intrinsics.K * P_camera_homogeneous;
    % 
    %     % Convert from homogeneous to 2D coordinates
    %     projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    %     projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    % 
    %     % Extract the 2D points
    %     u2(j) = projected_points(1, :);
    %     v2(j) = projected_points(2, :);
    % 
    %     e2(j) = abs([u2(j) v2(j) 1]* C_win{i} * [u2(j); v2(j); 1]);
    % end

    % if i == 2
    %     tstampStr = sprintf('%.0f', tstamp{i});
    %     image = strcat(imagePath, tstampStr, '.png');
    %     imshow(image); hold on;
    %     plot(u1, v1, 'b', 'MarkerSize',5, 'LineWidth', 4); hold on;% Plot reprojected points in red
    %     plot(u2, v2, 'r', 'MarkerSize',5, 'LineWidth', 4); % Plot reprojected points in red
    %     pause(1);
    % end
    % tstampStr = sprintf('%.0f', tstamp{i});
    % image = strcat(imagePath, tstampStr, '.png');
    % imshow(image); hold on;
    % plot(u1, v1, 'b', 'MarkerSize',5, 'LineWidth', 4); hold on;% Plot reprojected points in red
    % plot(u2, v2, 'r', 'MarkerSize',5, 'LineWidth', 4); % Plot reprojected points in red
    % pause(0.5);
end
% 
% ransac2 = logical(ransac2);
% tstamp = tstamp(ransac2);
% % imgList = imgList(ransac2, :);
% R_iss_cam_true = R_iss_cam_true(ransac2);
% t_iss_cam_true = t_iss_cam_true(ransac2);
% T_iss_cam_true = T_iss_cam_true(ransac2);
% T_p2c_cam = T_p2c_cam(ransac2);
% T_p2c_cam_true = T_p2c_cam_true(ransac2);
% T_cam_p2c = T_cam_p2c(ransac2);
% ellipses_params = ellipses_params(ransac2);
% PM_normal_cam = PM_normal_cam(ransac2);
% PM_center_cam = PM_center_cam(ransac2);
% Window_normal_cam = Window_normal_cam(ransac2);
% Window_center_cam = Window_center_cam(ransac2);
% C_pm = C_pm(ransac2);
% C_win = C_win(ransac2);
% EstimatedPose = EstimatedPose(ransac2);
% tracking_num = size(R_iss_cam_true, 2);
%% Step 2-4: Visualize Optimization Result
figure;
% plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
% plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 4); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 4);

for i = 1:tracking_num
    if i == 1
        plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
        plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
        plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
        view(-10, -45); 
        % view(0, 0); 
        grid on; axis equal;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title("Tracking Optimization");
        % pause(4);
    end
    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on; % true cam pose h
    % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    view(-10, -45); 
    % view(0, 0); 
    % plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 0.5, 'm');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("Tracking Optimization");
    pause(0.5);
end
set(gcf, 'Color', 'w');

grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
% ylim([-0.2 0.8]);
% zlim([0 2.1]);
% xlim([-0.5 0.5]);
title("Tracking Optimization");
f=FigureRotator();
k=10;

%% Step 2-5: Eval (ATE, ARE)
% [RPE_RMSE_BA,RPE_BA] = calcRPE(BAEstimatedPose, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_BA,ATE_BA] = calcATE(BAEstimatedPose, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_BA,RMD_BA] = calcRMD(BAEstimatedPose, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_MO,RPE_MO] = calcRPE(EstimatedPose, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_MO,ATE_MO] = calcATE(EstimatedPose, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_MO,RMD_MO] = calcRMD(EstimatedPose, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_p2c,RPE_p2c] = calcRPE(T_p2c_cam, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_p2c,ATE_p2c] = calcATE(T_p2c_cam, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_p2c,RMD_p2c] = calcRMD(T_p2c_cam, T_p2c_cam_true, 'RMSE');

% [RPE_RMSE_droid,RPE_droid] = calcRPE(T_p2c_droidcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_droid,ATE_droid] = calcATE(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_droid,RMD_droid] = calcRMD(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% 
% [RPE_RMSE_colmap,RPE_colmap] = calcRPE(T_p2c_colmapcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_colmap,ATE_colmap] = calcATE(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_colmap,RMD_colmap] = calcRMD(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');

reprojection_error_total = ( rms(e1) + rms(e2) ) / 2;
reprojection_error1 = rms(e1);
reprojection_error2 = rms(e2);

% fprintf("BA result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_BA, RPE_RMSE_BA, RMD_RMSE_BA);
fprintf("Motion Opt result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_MO, RPE_RMSE_MO, RMD_RMSE_MO);
fprintf("p2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2c, RPE_RMSE_p2c, RMD_RMSE_p2c);
% fprintf("DROID result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_droid, RPE_RMSE_droid, RMD_RMSE_droid);
% fprintf("COLMAP result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_colmap, RPE_RMSE_colmap, RMD_RMSE_colmap);
fprintf("reprojection error result \n PM: %f \n Win %f \n total %f \n", rms(e1), rms(e2), reprojection_error_total);


%% Functions
function ellipse_params = fit_ellipse(points_2D)
    % Fit an ellipse to 2D projected points using PCA (considering image plane orientation)
    % Returns [x_center, y_center, major_axis_length, minor_axis_length, theta]

    % Compute centroid (ellipse center)
    x_center = mean(points_2D(:,1));
    y_center = mean(points_2D(:,2));
    
    % Centralize points
    centralized_points = points_2D - [x_center, y_center];
    
    % Compute covariance matrix
    covariance_matrix = cov(centralized_points);
    
    % Compute eigenvectors and eigenvalues
    [eig_vecs, eig_vals] = eig(covariance_matrix);
    
    % Extract eigenvalues (variance along principal axes)
    eigenvalues = diag(eig_vals);
    [max_eigenvalue, max_index] = max(eigenvalues);
    min_eigenvalue = min(eigenvalues);
    
    % Major and minor axis lengths (2 * std deviation)
    major_axis_length = 2 * sqrt(max_eigenvalue);
    minor_axis_length = 2 * sqrt(min_eigenvalue);
    
    % Adjusting the eigenvector direction for image plane
    major_axis_vector = eig_vecs(:, max_index);
    
    % Compute rotation angle (theta) in the image plane
    % atan2d is used to get the correct quadrant; Y-axis is inverted in image plane
    theta = atan2d(major_axis_vector(2), -major_axis_vector(1)); 

    % Store ellipse parameters
    ellipse_params = [y_center, x_center, major_axis_length, minor_axis_length, 90+theta];
end

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


%%
function residuals = EReprojection_motiononly3(params, PM_Point3D, Win_Point3D, Qpm, Qwin, ellipses_params,PM_center_cam , K)
    % INPUTS:
    %  params        - 6x1 vector in se(3) (rotation + translation)
    %  InitialMap_* - your data structures for points, etc. 
    %  C_pm, C_win   - cell arrays or sets of ellipse (Q) matrices for each feature
    %  K             - intrinsic matrix (3x3)

    %  ellipse_parame1 - (x,y) 2nd frame (constant) :MINJI_0407_2
    %  PM_center_cam  - 1st frame back projection circle center :MINJI_0407_2
    %
    % OUTPUT:
    %  residuals - Nx1 vector of residuals, one per feature
    global final_T;

    % 1. Convert params (se(3)) to a 4x4 transform T
    T = expSE3(params);  % from the code above

    % Suppose you have N correspondences.
    % Example: let's assume you have 3D points in the first camera frame or
    % "world" frame => P_i. We want to reproject them into the second view.
    % Alternatively, you might have 2D points in the first view. 
    %   -> Adjust the lines below to your scenario.

    residuals = zeros(10, 1); 

    N = size(PM_Point3D, 2);
    % residuals = zeros(N,1);
    % lamda_center = 1;
    % lamda_conic  = 1 / N;
    circle2Dpoints_PM=zeros(N,2);
    circle2Dpoints_Window=zeros(N,2);


    % Example: We loop over each feature
    for i = 1:N

        % ----- 2D or 3D retrieval of the i-th feature -----
        % Let's say P_i is a 3D point in the first camera's frame
        % P_i = InitialMap_PM(i,:)';  % [X; Y; Z]
        pm_point3D = PM_Point3D(:,  i);
        win_point3D = Win_Point3D(:, i);
        
        % Convert to homogeneous 4D:
        % P_i_h = [P_i; 1];
        pm_point3D_h = [pm_point3D; 1];
        win_point3D_h = [win_point3D; 1];

        % 2. Transform this point into the second camera's frame
        %    T: 2nd_cam_frame <- 1st_cam_frame
        % P2_i_h = T * pm_point3D_h;
        % P2_i = P2_i_h(1:3);  % [X2; Y2; Z2]
        pm_P2_i_h = T*pm_point3D_h;
        win_P2_i_h = T*win_point3D_h;
        pm_P2_i = pm_P2_i_h(1:3);
        win_P2_i = win_P2_i_h(1:3);

        % 3. Project using intrinsics K
        % x2_h = K * P2_i;    % homogeneous 2D = [u; v; w]
        pm_x2_h = K * pm_P2_i;
        win_x2_h = K * win_P2_i;
        % Normalize to get pixel coords [u/w; v/w; 1]
        % x2_h = x2_h / x2_h(3);
        pm_x2_h = pm_x2_h / pm_x2_h(3);
        win_x2_h = win_x2_h / win_x2_h(3);
        circle2Dpoints_PM(i,:)=pm_x2_h(1:2)';
        circle2Dpoints_Window(i,:)=win_x2_h(1:2)';
    end

    
    ellipse_result1_after = fit_ellipse(circle2Dpoints_PM);
    ellipse_result2_after = fit_ellipse(circle2Dpoints_Window);
    ellipses_params_after = [ellipse_result1_after; ellipse_result2_after];
    
    for i = 1:2
        % 파라미터 추출
        k1 = ellipses_params_after(i, 1);  % y_center
        h1 = ellipses_params_after(i, 2);  % x_center
        a1 = ellipses_params_after(i, 3);  % major axis
        b1 = ellipses_params_after(i, 4);  % minor axis
        theta1 = ellipses_params_after(i, 5);  % orientation (deg)

        k2 = ellipses_params(i, 1);
        h2 = ellipses_params(i, 2);
        a2 = ellipses_params(i, 3);
        b2 = ellipses_params(i, 4);
        theta2 = ellipses_params(i, 5);

        % 각도 차이 modulo 처리
        dtheta = mod(theta1 - theta2 + 180, 360) - 180;

        % residual vector에 차이 저장
        residuals((i-1)*5+1 : i*5) = [
            h1 - h2;
            k1 - k2;
            a1 - a2;
            b1 - b2;
            dtheta
            ];
    end
   

    % k = ellipse_result1_after(1, 1); %y
    % h = ellipse_result1_after(1, 2); %x
    % a = ellipse_result1_after(1, 3)/2; %a
    % b = ellipse_result1_after(1, 4)/2; %b
    % angle = 90-ellipse_result1_after(1, 5); %angle (deg)
    % % ellipse equation to quadratic form
    % [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
    % C_pm= [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F];
    % k1 = ellipse_result2_after(1, 1); %y
    % h1 = ellipse_result2_after(1, 2); %x
    % a1 = ellipse_result2_after(1, 3)/2; %a
    % b1 = ellipse_result2_after(1, 4)/2; %b
    % angle1 = 90-ellipse_result2_after(1, 5); %angle (deg)
    % ellipse equation to quadratic form
    % [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
    % C_win= [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F];
    % 
    % 
    % residuals = 1000*(norm(Qpm - C_pm)^2 + norm(Qwin-C_win)^2);

    % image_size = [880 1280]; 
    % figure; hold on; axis equal;
    % xlim([0, image_size(2)]); ylim([0, image_size(1)]);
    % set(gca, 'YDir', 'reverse');  % Invert the y-axisfigure;
    % hold on;
    % axis equal;
    % grid on;
    % 
    % % PM 점들 (빨간 동그라미)
    % scatter(circle2Dpoints_PM(:,1), circle2Dpoints_PM(:,2), 10, 'r', 'filled');
    % scatter(circle2Dpoints_Window(:,1), circle2Dpoints_Window(:,2), 10, 'b', 'filled');
    % 
    % 
    % legend('PM points','Window points');
    % title('2D Points from PM and Window');
    % xlabel('X');
    % ylabel('Y');

    % 4. Suppose C_win(i) is the ellipse Q_i in the 2D image
    %    We want x2^T * Q_i * x2 = 0.
    %    Format x2 as [u; v; 1], and Q_i is 3x3.

    % Evaluate the conic constraint
    % val_pm = pm_x2_h' * Qpm * pm_x2_h;
    % val_win = win_x2_h' * Qwin * win_x2_h;

    % 5. That val is the i-th residual
    %residuals(i) = val_pm + val_win;
    % residuals(i) = (val_pm + val_win);
    %end

    final_T = T;
    % disp(mean(residuals));

    % %===Add center point residual ==%
    % PM_center_cam1=PM_center_cam{1};
    % %PM_center_cam, Window_center_cam : 1 frame에서 backprojection Circle's 3D (x,y,z) => projection to 2nd frame 
    % pm_center_h = [PM_center_cam1; 1];
    % %win_center_h = [Window_center_cam; 1];
    % 
    % % Pose transformation (1st frame → 2nd frame)
    % pm_center_trans = T * pm_center_h;
    % %win_center_trans = T * win_center_h;
    % 
    % % 3D → 2D projection with intrinsics
    % pm_center_proj = K * pm_center_trans(1:3);
    % %win_center_proj = K * win_center_trans(1:3);
    % 
    % % Normalize to pixel coordinates
    % pm_center_proj = pm_center_proj / pm_center_proj(3);
    % %win_center_proj = win_center_proj / win_center_proj(3);
    % 
    % 
    % 
    % %2nd frame의 ellipse param (x, y)
    % x1 = ellipses_params(1, 2); 
    % y1 = ellipses_params(1, 1); 
    % %next frame detected ellipse
    % reference_center=[x1;y1;1];
    % 
    % % Euclidean distance in image
    % % 대각선 길이
    % residuals(end+1) = lamda_center*(sqrt(mean((pm_center_proj(1:2)-reference_center(1:2)).^2)));
    % 

    % If you also have constraints in the first image or other constraints,
    % you can stack them in the residual vector as well.

end

%%

function F = EReprojection_localmapping(params, tracking_num, sample_num, Qpm, Qwin, K)
    Rk = cell(1, tracking_num);
    Tk = cell(1, tracking_num);
    for i = 0:tracking_num-1
        roll = params(1);
        pitch = params(2);
        yaw = params(3);
        Rk = angle2dcm( yaw, pitch, roll );
        Tk = params(4:6)';

        Ti = params(1:3, i*4-3:i*4);
        Rk{i} = Ti(1:3, 1:3);
        Tk{i} = Ti(1:3, 4);
    end
    PM_Point3D = params(1:3, 4*(tracking_num-1)+1:4*(tracking_num-1)+sample_num);
    Win_Point3D = params(1:3, 4*(tracking_num-1)+sample_num+1:end);


    n = size(PM_Point3D, 2);
    F1 = zeros(sample_num, 1);
    F2 = zeros(sample_num, 1);
    Ft = zeros(tracking_num, 1);
    for i = 1:tracking_num-1
        for j = 1:sample_num
            % 3D point
            pm_point3D = PM_Point3D(:, j);
            win_point3D = Win_Point3D(:, j);
    
            pm_projected_point = K * (Rk{i} * pm_point3D + Tk{i});
            pm_projected_point = pm_projected_point(1:2) / pm_projected_point(3);
            pm_error = [pm_projected_point; 1]' * Qpm{i+1} * [pm_projected_point; 1];
    
            win_projected_point = K * (Rk{i} * win_point3D + Tk{i});
            win_projected_point = win_projected_point(1:2) / win_projected_point(3);
            win_error = [win_projected_point; 1]' * Qwin{i+1} * [win_projected_point; 1];
    
            F1(j) = pm_error;
            F2(j) = win_error;
        end
        Ft(i) = sum(F1) + sum(F2);
    end
    F = sum(Ft);
end


function F = EReprojectionBackprojection_localmapping(params, tracking_num, sample_num, Qpm, Qwin, PM_normal_cam, PM_center_cam, Window_normal_cam, Window_center_cam, K)
    Rk = cell(1, tracking_num);
    Tk = cell(1, tracking_num);
    Ti = cell(1, tracking_num);
    for i = 1:tracking_num
        % Ti{i} = [params(1:3, i*4-3:i*4); 0 0 0 1];
        % Rk{i} = Ti{i}(1:3, 1:3);
        % Tk{i} = Ti{i}(1:3, 4);
        qk{i} = params(1, i*4-3:i*4);
        Tk{i} = params(2, i*4-3:i*4-1);
        Rk{i} = quat2rotm(qk{i});
    end
    PM_Point3D = params(1:3, 4*(tracking_num-1)+1:4*(tracking_num-1)+sample_num);
    Win_Point3D = params(1:3, 4*(tracking_num-1)+sample_num+1:end);


    n = size(PM_Point3D, 2);
    F1 = zeros(sample_num, 1);
    F2 = zeros(sample_num, 1);
    Fr = zeros(tracking_num-1, 1);
    Fb = zeros(tracking_num-1, 1);
    for i = 1:tracking_num-1
        % Reprojection Error
        for j = 1:sample_num
            % 3D point
            pm_point3D = PM_Point3D(:, j);
            win_point3D = Win_Point3D(:, j);
    
            pm_projected_point = K * (Rk{i} * pm_point3D + Tk{i});
            pm_projected_point = pm_projected_point(1:2) / pm_projected_point(3);
            pm_error = [pm_projected_point; 1]' * Qpm{i+1} * [pm_projected_point; 1];
    
            win_projected_point = K * (Rk{i} * win_point3D + Tk{i});
            win_projected_point = win_projected_point(1:2) / win_projected_point(3);
            win_error = [win_projected_point; 1]' * Qwin{i+1} * [win_projected_point; 1];
    
            F1(j) = pm_error;
            F2(j) = win_error;
        end
        Fr(i) = sum(F1) + sum(F2);

        % Backprojection Error
        PM_next = inv(Ti{i}) * [PM_normal_cam{i+1}; 1];
        Fb(i) = abs(abs(dot(PM_normal_cam{1}, PM_next(1:3))) - 1);
    end
    F = sum(Fr) + sum(Fb);
end

function projected_points = project_3D_to_2D(points_3D, K, camera_pose)
    % Input:
    % points_3D: Nx3 matrix of 3D points (X, Y, Z) in world coordinates
    % K: 3x3 intrinsic matrix of the camera
    % camera_pose: 4x4 transformation matrix (T_wc) from camera to world
    % Output:
    % projected_points: Nx2 matrix of pixel coordinates

    % Compute the inverse pose to get world-to-camera transformation
    T_cw = inv(camera_pose);  % Now it transforms world points to the camera frame
    
    % Extract rotation (R) and translation (t) from T_cw
    R = T_cw(1:3, 1:3);
    t = T_cw(1:3, 4);
    
    % Convert 3D points from world to camera coordinates
    points_camera = (R * points_3D' + t)'; % Nx3 matrix
    
    % Keep only points in front of the camera (Z > 0)
    valid_idx = points_camera(:, 3) > 0;
    points_camera = points_camera(valid_idx, :);

    % Project to 2D using intrinsics
    points_homogeneous = K * points_camera'; % 3xN matrix
    
    % Normalize to get pixel coordinates
    projected_points = points_homogeneous(1:2, :) ./ points_homogeneous(3, :);
    
    % Transpose to return Nx2 format
    projected_points = projected_points';
end

function T = expSE3(xi)
% xi is 6x1, [w1; w2; w3; v1; v2; v3]
% Returns a 4x4 homogeneous transformation T = exp( [w]x  v; 0 0 0  )

    % Split rotation/translation
    omega = xi(1:3);
    v     = xi(4:6);

    theta = norm(omega);
    if theta < 1e-12
        % If very small rotation, approximate R = I + [w]x
        R = eye(3) + skew3(omega);
        J = eye(3);  % Approx. for small angles
    else
        k = omega / theta;
        K = skew3(k);
        % Rodrigues' formula for rotation
        R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);

        % The "Jacobian" that appears in SE(3) exponential
        J = eye(3) + (1 - cos(theta))/theta * K + ...
            (theta - sin(theta))/theta * (K*K);
    end

    t = J * v;  % translation part

    % Build the 4x4 homogeneous transform
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4)   = t;
end
function xi = logSE3(T)
% logSE3: SE(3) -> se(3)
%   T: 4x4 matrix in SE(3)
%   xi: 6x1 = [omega; v]
    R = T(1:3,1:3);
    t = T(1:3,4);

    % Rotation part
    omega = logSO3(R);
    theta = norm(omega);

    if theta < 1e-12
        % near zero rotation => v = t
        v = t;
    else
        k = omega / theta;
        K = skew3(k);
        % The "V" matrix from Rodrigues
        V = eye(3) + ((1 - cos(theta))/theta)*K + ...
                  ((theta - sin(theta))/theta)*(K*K);
        v = V \ t;  % or inv(V)*t
    end
    xi = [omega; v];
end

function omega = logSO3(R)
% logSO3: SO(3) -> so(3)
%   R: 3x3 rotation matrix
%   omega: 3x1 axis-angle
    cosTheta = (trace(R) - 1)/2;
    cosTheta = max(min(cosTheta,1),-1);  % clamp for numerical safety
    theta = acos(cosTheta);
    
    if abs(theta) < 1e-12
        % near identity
        omega = skewInv(R - eye(3));
    else
        w_hat = (R - R')/(2*sin(theta));
        w = skewInv(w_hat);
        omega = w * theta;
    end
end

function S = skew3(v)
% skew3: 3x1 -> 3x3 skew-symmetric
    S = [   0    -v(3)   v(2);
          v(3)     0    -v(1);
         -v(2)   v(1)     0   ];
end

function v = skewInv(S)
% skewInv: 3x3 skew-symmetric -> 3x1 vector
    v = [ S(3,2); S(1,3); S(2,1) ];
end