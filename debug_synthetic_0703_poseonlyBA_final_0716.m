% Clear workspace
clear; clc; close all;
% posePath = 'data/td_yaw/groundtruth_selected.txt';
% imagePath = 'data/td_yaw/gray_selected/';

% posePath = 'data/ff_return_journey_forward/groundtruth_selected.txt';
% imagePath = 'data/ff_return_journey_forward/gray_selected/';
posePath = 'data/multiview_mixed_data/groundtruth.txt';
% imagePath = 'data/synthetic/';
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

figure; hold on; axis equal;
grid on; view(45, 45);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Visualization of Circles and Virtual Camera');
% Plot the circles in 3D
plot3(circle1_points_3D(1, :), circle1_points_3D(2, :), circle1_points_3D(3, :), 'r-', 'LineWidth', 1.5); hold on;
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
for i=1:tracking_num
    plot_camera_frame(R_iss_cam_true{i}, t_iss_cam_true{i}, 0.5, 'r'); hold on;
    % pause(0.5);
    plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
end

legend('Circle 1', 'Circle 2', 'Circle 1 Center', 'Circle 2 Center', 'Circle 1 Normal', 'Circle 2 Normal', ...
       'Camera Position', 'Camera Orientation');
hold off;
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
% for i = 1:tracking_num
%     x1 = circle1_points_2D{i}(:,1);
%     y1 = circle1_points_2D{i}(:,2);
%     x2 = circle2_points_2D{i}(:,1);
%     y2 = circle2_points_2D{i}(:,2);
% 
%     ellipse_result1{i} = fit_ellipse_nonlinear(x1, y1);
%     ellipse_result2{i} = fit_ellipse_nonlinear(x2, y2);
% end
% figure; hold on; axis equal; set(gca, 'YDir', 'reverse');
% for i = 1:tracking_num
%     x1 = circle1_points_2D{i}(:, 1);
%     y1 = circle1_points_2D{i}(:, 2);
%     x2 = circle2_points_2D{i}(:, 1);
%     y2 = circle2_points_2D{i}(:, 2);
% 
% 
%     ellipse_result1{i}(1, :) = fit_ellipse_conic(x1, y1);
%     ellipse_result2{i}(1, :) = fit_ellipse_conic(x2, y2);
% 
% end


% Display results
% disp('Ellipse 1 Parameters (x_center, y_center, major_axis, minor_axis, theta):');
% disp(ellipse1);
% 
% disp('Ellipse 2 Parameters (x_center, y_center, major_axis, minor_axis, theta):');
% disp(ellipse2);
image = './data/ff_return_journey_forward/gray/1617290600939388160.png';

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
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);

    % Visualize Detected Ellipse
    % figure;
    % figure_detected_two_ellipse(image, ellipses_params{i}, '1617290600939388160'); hold on;
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

    z = ( PM_center_cam{i} - Window_center_cam{i});
    z = z / norm(z);
    x = cross(Window_normal_cam{i}, z);
    x = x / norm(x);
    z = cross(x, Window_normal_cam{i});
    y = Window_normal_cam{i};
    R_cam_w2c = [x, y, z];
    t_cam_w2c = Window_center_cam{i};
    T_cam_w2c{i} = [R_cam_w2c, t_cam_w2c; 0 0 0 1];

    
end
ransac = logical(ransac);
tstamp = tstamp(ransac);
% imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
% T_p2c_cam = T_p2c_cam(ransac);
%% 
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



% T_p2c_iss = T_p2c_cam{1} * inv(T_iss_cam_true{1});
% for i = 1:tracking_num
%     T_p2c_iss = T_p2c_cam{i} * inv(T_iss_cam_true{i});
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end


%% STEP 2 
% Visualize the cone
figure; 
hold on;
z_means = zeros(1, tracking_num);
InitialMap_PM_all = cell(1, tracking_num);
InitialMap_Win_all = cell(1, tracking_num);
valid_k_list = [];
valid_z_means = [];
for k=1:tracking_num
    % figure;
    % plot_inertial_frame(1); hold on; % Camera Frame
    for i = 1:2
        k1 = ellipses_params{k}(i, 1); %y
        h = ellipses_params{k}(i, 2); %x
        a = ellipses_params{k}(i, 3)/2; %a
        b = ellipses_params{k}(i, 4)/2; %b
        theta = 90-ellipses_params{k}(i, 5); %angle (deg)
        t = linspace(0, 2*pi, sample_num);
        x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
        y_ellipse = k1 + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
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
    if abs(PM_normal_cam{k}(1, 1)) < abs(PM_normal_cam{k}(2, 1)) && abs(PM_normal_cam{k}(1, 1)) < abs(PM_normal_cam{k}(3, 1))
        v = [1, 0, 0];
    elseif abs(PM_normal_cam{k}(2, 1)) < abs(PM_normal_cam{k}(1, 1)) && abs(PM_normal_cam{k}(2, 1)) < abs(PM_normal_cam{k}(3, 1))
        v = [0, 1, 0];
    else
        v = [0, 0, 1];
    end
    u = cross(PM_normal_cam{k}(:, 1), v);
    u = u / norm(u);
    v = cross(PM_normal_cam{k}(:, 1), u);
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
    plot3(PM_center_cam{k}(1, 1) + PM_circle_3D(1, :), PM_center_cam{k}(2, 1) + PM_circle_3D(2, :), PM_center_cam{k}(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;

    %Window circle
    if abs(Window_normal_cam{k}(1, 1)) < abs(Window_normal_cam{k}(2, 1)) && abs(Window_normal_cam{k}(1, 1)) < abs(Window_normal_cam{k}(3, 1))
        v = [1, 0, 0];
    elseif abs(Window_normal_cam{k}(2, 1)) < abs(Window_normal_cam{k}(1, 1)) && abs(Window_normal_cam{k}(2, 1)) < abs(Window_normal_cam{k}(3, 1))
        v = [0, 1, 0];
    else
        v = [0, 0, 1];
    end
    u = cross(Window_normal_cam{k}(:, 1), v);
    u = u / norm(u);
    v = cross(Window_normal_cam{k}(:, 1), u);
    theta = linspace(0, 2*pi, sample_num);
    Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
    plot3(Window_center_cam{k}(1, 1) + Window_circle_3D(1, :), Window_center_cam{k}(2, 1) + Window_circle_3D(2, :), Window_center_cam{k}(3, 1) + Window_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;
    InitialMap_PM_all{k} = [PM_center_cam{k}(1) + PM_circle_3D(1, :); ...
                            PM_center_cam{k}(2) + PM_circle_3D(2, :); ...
                            PM_center_cam{k}(3) + PM_circle_3D(3, :)];

    InitialMap_Win_all{k} = [Window_center_cam{k}(1) + Window_circle_3D(1, :); ...
                             Window_center_cam{k}(2) + Window_circle_3D(2, :); ...
                             Window_center_cam{k}(3) + Window_circle_3D(3, :)];
   
    % z_diff = (Window_center_cam{k}(3,1) + Window_circle_3D(3,:)) - ...
        % (PM_center_cam{k}(3,1) + PM_circle_3D(3,:));
    z_diff = (Window_center_cam{k}(3,1)) - (PM_center_cam{k}(3,1));

    z_means(k) = mean(z_diff);
    % z_means=z_diff;

end
[~, best_k] = min(abs(z_means));
disp(PM_center_cam{best_k});
disp(z_means);
disp(best_k);
% best_k=1;
disp(min(abs(z_means)));
InitialMap_PM = InitialMap_PM_all{best_k};
InitialMap_Win = InitialMap_Win_all{best_k};
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
% figure;
% plot3(InitialMap_PM_all{k}(1, :), InitialMap_PM_all{k}(2, :), InitialMap_PM_all{k}(3, :), 'bo', 'MarkerSize', 4); hold on;
% plot3(InitialMap_Win_all{k}(1, :), InitialMap_Win_all{k}(2, :), InitialMap_Win_all{k}(3, :), 'ro', 'MarkerSize', 4);
% 
% xlabel('X'); ylabel('Y'); zlabel('Z');
% 
% axis equal;
% grid on;
view(3); % 3D 시점

%% Change to global frame 
for i=1:tracking_num
    T_p2c_cam{i} = T_cam_p2c{best_k} * inv(T_cam_p2c{i});%PM기준
    % T_w2c_cam{i} = T_cam_w2c{best_k} * inv(T_cam_w2c{i});

end
T_p2c_iss = T_p2c_cam{best_k} * inv(T_iss_cam_true{best_k});
% T_w2c_iss = T_w2c_cam{best_k} * inv(T_iss_cam_true{best_k});

for i = 1:tracking_num
    T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
    % T_w2c_cam_true{i} = T_w2c_iss * T_iss_cam_true{i};
end




%% Tracking Optimization
disp("optimization");
ransac2 = ones(1, tracking_num);
EstimatedPose{best_k} = T_p2c_cam{best_k};
% EstimatedPose{best_k} = T_w2c_cam{best_k};
T_ref=EstimatedPose{best_k};
PM_normal_i = EstimatedPose{best_k} * [PM_normal_cam{best_k}; 1];
PM_normal_init = PM_normal_i(1:3)/norm(PM_normal_i(1:3));

pose_params_init = []; 	% BA 최적화에 들어갈 초기 pose 파라미터들 (logSE3 형식, 6D) // Size : 6×(tracking_num-1)
pose_index_map = [];    % 그 파라미터가 몇 번째 frame의 pose인지 기록하는 매핑 정보 //


C_pm_all = cell(1, tracking_num);
C_win_all = cell(1, tracking_num);

for i = 1:tracking_num
    C_pm_all{i} = C_pm{i};
    C_win_all{i} = C_win{i};
end



for i =1:tracking_num
    if i==best_k
        continue
    end

    % T_relative = T_w2c_cam{best_k};
    T_relative = inv( inv(T_p2c_cam{best_k}) * T_p2c_cam{i} ); %  frame i의 pose를 best_k 기준으로 상대 pose로 변환
    xi = logSE3(T_relative); % SE(3) 변환 행렬을 6D vector로 바꿈 (Lie algebra)
    pose_params_init = [pose_params_init; xi]; % 위에서 계산한 6D pose 파라미터를 최적화 초기값에 쌓음
    pose_index_map(end+1) = i; % 현재 이 xi가 몇 번째 frame의 pose인지 기록 (예: 2번, 4번, ...)

end

objective_function=@(params) BA_motiononly_cost( ...
    params, InitialMap_PM, InitialMap_Win, C_pm_all,C_win_all,intrinsics.K,pose_index_map);

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
    'Display','iter','FiniteDifferenceType','forward');

[opt_params, resnorm] = lsqnonlin(objective_function, pose_params_init, [], [], options);

for cnt = 1:length(pose_index_map) % 적화 대상이었던 frame들만 loop을 돔 (예: best_k 제외한 나머지)
    i = pose_index_map(cnt);
    xi = opt_params((cnt-1)*6+1 : cnt*6); % 최적화된 6D pose vector (logSE3)를 추출
    T_rel = expSE3(xi); % 6D vector를 다시 SE(3) 행렬로 복원
    EstimatedPose{i} = EstimatedPose{best_k} * inv(T_rel);
    R_relative_opt{i} = T_rel(1:3, 1:3);
    T_relative_opt{i} = T_rel(1:3, 4);

end


for i = 1:tracking_num
    if isempty(EstimatedPose{i})
        continue
    end

    PM_normal = EstimatedPose{i} * [PM_normal_cam{i}; 1];
    PM_normal_esti = PM_normal(1:3) / norm(PM_normal(1:3));
    angle_diff2{i} = acos(dot(PM_normal_init, PM_normal_esti)) * 180 / pi;

    if angle_diff2{i} > 300
        ransac2(i) = 0;
    end
end

ransac2 = logical(ransac2);
% tstamp = tstamp(ransac2);
% imgList = imgList(ransac2, :);
R_iss_cam_true = R_iss_cam_true(ransac2);
t_iss_cam_true = t_iss_cam_true(ransac2);
T_iss_cam_true = T_iss_cam_true(ransac2);
T_p2c_cam = T_p2c_cam(ransac2);
% T_w2c_cam = T_w2c_cam(ransac2);
T_p2c_cam_true = T_p2c_cam_true(ransac2);
T_cam_p2c = T_cam_p2c(ransac2);
ellipses_params = ellipses_params(ransac2);
PM_normal_cam = PM_normal_cam(ransac2);
PM_center_cam = PM_center_cam(ransac2);
Window_normal_cam = Window_normal_cam(ransac2);
Window_center_cam = Window_center_cam(ransac2);
C_pm = C_pm(ransac2);
C_win = C_win(ransac2);
EstimatedPose = EstimatedPose(ransac2);
tracking_num = size(R_iss_cam_true, 2);
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
        % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
        view(-10, -45); 
        % view(0, 0); 
        grid on; axis equal;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title("Tracking Optimization");
        % pause(4);
    end
    % plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
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




%% Visualize Optimization Result
figure;
% plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
% plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);

for i = 1:tracking_num
    transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);

    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    % plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    view(-10, -45);
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    title("Tracking Optimization");
end
set(gcf, 'Color', 'w');

grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Tracking Optimization");
f=FigureRotator();
k=10;
hold off;


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
% 
% [RPE_RMSE_p2cw,RPE_p2cw] = calcRPE(T_w2c_cam, T_w2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_p2cw,ATE_p2cw] = calcATE(T_w2c_cam, T_w2c_cam_true, 'RMSE');
% [RMD_RMSE_p2cw,RMD_p2cw] = calcRMD(T_w2c_cam, T_w2c_cam_true, 'RMSE');

% [RPE_RMSE_droid,RPE_droid] = calcRPE(T_p2c_droidcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_droid,ATE_droid] = calcATE(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_droid,RMD_droid] = calcRMD(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% 
% [RPE_RMSE_colmap,RPE_colmap] = calcRPE(T_p2c_colmapcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_colmap,ATE_colmap] = calcATE(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_colmap,RMD_colmap] = calcRMD(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');

% reprojection_error_total = ( rms(e1) + rms(e2) ) / 2;
% reprojection_error1 = rms(e1);
% reprojection_error2 = rms(e2);

fprintf("Motion Opt result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_MO, RPE_RMSE_MO, RMD_RMSE_MO);
fprintf("p2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2c, RPE_RMSE_p2c, RMD_RMSE_p2c);
% fprintf("w2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2cw, RPE_RMSE_p2cw, RMD_RMSE_p2cw);


%% Functions


function residuals = BA_motiononly_cost(params, PM3D, WIN3D, C_pm_all, C_win_all, K,  pose_index_map)
    N = length(pose_index_map);
    

    % Reconstruct all poses from parameters
    for cnt = 1:N
        xi = params((cnt-1)*6+1 : cnt*6);
        T_rel = expSE3(xi);
        idx = pose_index_map(cnt);
        Pose{idx} = T_rel;
        
    end

    % Compute residuals
    residuals = [];

    for i = 1:length(Pose)
        if isempty(Pose{i})
            continue
        end
        % image = strcat(imagePath, imgList(i, :));

        Qpm = C_pm_all{i};
        Qwin = C_win_all{i};




        for j = 1:size(PM3D, 2)
            pm_point3D = PM3D(:, j);
            win_point3D = WIN3D(:, j);
            pm_point3D_h = [pm_point3D; 1];
            win_point3D_h = [win_point3D; 1];

            pm_P2_i_h = Pose{i}*pm_point3D_h;
            win_P2_i_h = Pose{i}*win_point3D_h;
            pm_P2_i = pm_P2_i_h(1:3);
            win_P2_i = win_P2_i_h(1:3);

            pm_x2_h = K * pm_P2_i;
            win_x2_h = K * win_P2_i;
            pm_x2_h = pm_x2_h / pm_x2_h(3);
            win_x2_h = win_x2_h / win_x2_h(3);
            val_pm = pm_x2_h' * Qpm * pm_x2_h;
            val_win = win_x2_h' * Qwin * win_x2_h;
            residuals = [residuals; val_pm+val_win];
        end
    end

end



function residuals = EReprojection_motiononly3(params, PM_Point3D, Win_Point3D, Qpm, Qwin, K)
    % INPUTS:
    %  params        - 6x1 vector in se(3) (rotation + translation)
    %  InitialMap_* - your data structures for points, etc. 
    %  C_pm, C_win   - cell arrays or sets of ellipse (Q) matrices for each feature
    %  K             - intrinsic matrix (3x3)
    %
    % OUTPUT:
    %  residuals - Nx1 vector of residuals, one per feature

    % 1. Convert params (se(3)) to a 4x4 transform T
    T = expSE3(params);  % from the code above

    % Suppose you have N correspondences.
    % Example: let's assume you have 3D points in the first camera frame or
    % "world" frame => P_i. We want to reproject them into the second view.
    % Alternatively, you might have 2D points in the first view. 
    %   -> Adjust the lines below to your scenario.

    N = size(PM_Point3D, 2);
    residuals = zeros(N,1);
    % global all_residuals_list

    % % ellipse center point euclidian distance
    % x1 = ellipse_param1(2);
    % y1 = ellipse_param1(1);
    % first_center = [x1; y1; 1];
    % x2 = ellipse_param2(2);
    % y2 = ellipse_param2(1);
    % next_center = [x2; y2; 1];
    % Qpm = Qpm / norm(Qpm, 'fro');  % 또는 Qpm = Qpm / Qpm(3,3);
    % Qwin = Qwin / norm(Qwin, 'fro');



    % Example: We loop over each feature
    for i = 1:N

        % ----- 2D or 3D retrieval of the i-th feature -----
        % Let's say P_i is a 3D point in the first camera's frame
        % P_i = InitialMap_PM(i,:)';  % [X; Y; Z]
        pm_point3D = PM_Point3D(:, i);
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

        % 4. Suppose C_win(i) is the ellipse Q_i in the 2D image
        %    We want x2^T * Q_i * x2 = 0.
        %    Format x2 as [u; v; 1], and Q_i is 3x3.
        


        % Evaluate the conic constraint
        val_pm = pm_x2_h' * Qpm * pm_x2_h;
        val_win = win_x2_h' * Qwin * win_x2_h;
        % % disp(1000*[val_pm, val_win]);
        % disp([val_pm, val_win]);
        % disp(Qpm);
        % disp(Qwin);

        % 5. That val is the i-th residual
        residuals(i) = val_pm + val_win;
        %residuals(i) = 1000*(abs(val_pm) + abs(val_win));
        % residuals(i) = sqrt(100*(val_pm)^2 +10*(val_win)^2);
        % log_val_pm(i) = log10(abs(val_pm));
        % log_val_win(i) = log10(abs(val_win));

        
        %fprintf("i=%d, residual = %.10f\n", i, residuals(i));
    end
    % if exist('log_val_pm', 'var')
    %     figure;
    %     plot(log_val_pm);
    %     title('log_{10}(abs(val_{pm}))');
    %     xlabel('Feature Index');
    %     ylabel('Log Value');
    % 
    %     figure;
    %     plot(log_val_win);
    %     title('log_{10}(abs(val_{win}))');
    %     xlabel('Feature Index');
    %     ylabel('Log Value');
    % % end
    % all_residuals_list{end+1} = residuals;



end


function res = ellipse_residual(p, xy)
    x0 = p(1); y0 = p(2); a = p(3); b = p(4); theta = p(5);
    cos_t = cos(theta); sin_t = sin(theta);
    dx = xy(:,1) - x0; dy = xy(:,2) - y0;
    xt = dx * cos_t + dy * sin_t;
    yt = dx * sin_t - dy * cos_t;
    res = (xt./a).^2 + (yt./b).^2 - 1;
end
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
    scale=1.4;
    % Major and minor axis lengths (2 * std deviation)
    major_axis_length = 2 *scale* sqrt(max_eigenvalue);
    minor_axis_length = 2 *scale* sqrt(min_eigenvalue);
    
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

% 
% function residuals = EReprojection_motiononly3(params, PM_Point3D, Win_Point3D, Qpm, Qwin, K)
%     % INPUTS:
%     %  params        - 6x1 vector in se(3) (rotation + translation)
%     %  InitialMap_* - your data structures for points, etc. 
%     %  C_pm, C_win   - cell arrays or sets of ellipse (Q) matrices for each feature
%     %  K             - intrinsic matrix (3x3)
%     %
%     % OUTPUT:
%     %  residuals - Nx1 vector of residuals, one per feature
% 
%     % 1. Convert params (se(3)) to a 4x4 transform T
%     T = expSE3(params);  % from the code above
% 
%     % Suppose you have N correspondences.
%     % Example: let's assume you have 3D points in the first camera frame or
%     % "world" frame => P_i. We want to reproject them into the second view.
%     % Alternatively, you might have 2D points in the first view. 
%     %   -> Adjust the lines below to your scenario.
% 
%     N = size(PM_Point3D, 2);
%     residuals = zeros(N,1);
% 
%     % % ellipse center point euclidian distance
%     % x1 = ellipse_param1(2);
%     % y1 = ellipse_param1(1);
%     % first_center = [x1; y1; 1];
%     % x2 = ellipse_param2(2);
%     % y2 = ellipse_param2(1);
%     % next_center = [x2; y2; 1];
% 
% 
% 
%     % Example: We loop over each feature
%     for i = 1:N
% 
%         % ----- 2D or 3D retrieval of the i-th feature -----
%         % Let's say P_i is a 3D point in the first camera's frame
%         % P_i = InitialMap_PM(i,:)';  % [X; Y; Z]
%         pm_point3D = PM_Point3D(:, i);
%         win_point3D = Win_Point3D(:, i);
% 
%         % Convert to homogeneous 4D:
%         % P_i_h = [P_i; 1];
%         pm_point3D_h = [pm_point3D; 1];
%         win_point3D_h = [win_point3D; 1];
% 
%         % 2. Transform this point into the second camera's frame
%         %    T: 2nd_cam_frame <- 1st_cam_frame
%         % P2_i_h = T * pm_point3D_h;
%         % P2_i = P2_i_h(1:3);  % [X2; Y2; Z2]
%         pm_P2_i_h = T*pm_point3D_h;
%         win_P2_i_h = T*win_point3D_h;
%         pm_P2_i = pm_P2_i_h(1:3);
%         win_P2_i = win_P2_i_h(1:3);
% 
%         % 3. Project using intrinsics K
%         % x2_h = K * P2_i;    % homogeneous 2D = [u; v; w]
%         pm_x2_h = K * pm_P2_i;
%         win_x2_h = K * win_P2_i;
%         % Normalize to get pixel coords [u/w; v/w; 1]
%         % x2_h = x2_h / x2_h(3);
%         pm_x2_h = pm_x2_h / pm_x2_h(3);
%         win_x2_h = win_x2_h / win_x2_h(3);
% 
%         % 4. Suppose C_win(i) is the ellipse Q_i in the 2D image
%         %    We want x2^T * Q_i * x2 = 0. 
%         %    Format x2 as [u; v; 1], and Q_i is 3x3.
% 
%         % Evaluate the conic constraint
%         val_pm = pm_x2_h' * Qpm * pm_x2_h;  
%         val_win = win_x2_h' * Qwin * win_x2_h; 
% 
%         % 5. That val is the i-th residual
%         residuals(i) = val_pm + val_win;
%     end
% 
%     % If you also have constraints in the first image or other constraints,
%     % you can stack them in the residual vector as well.
% 
% end

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

