%% Optitrack data
clear; clc; close all;
imagePath = 'data_optitrack/data/td_roll_pitch_yaw/gray/';
ellipsePath = 'data_optitrack/data/td_roll_pitch_yaw/results_p2c/';
posePath = 'data_optitrack/data/td_roll_pitch_yaw/groundtruth.txt';
% imagePath = 'data_optitrack/data/sfm/gray/';
% ellipsePath = 'data_optitrack/data/sfm/results_p2c/';
% posePath = 'data_optitrack/data/sfm/groundtruth.txt';
% imagePath = 'data_optitrack/data/ff_return_journey_forward/gray/';
% ellipsePath = 'data_optitrack/data/ff_return_journey_forward/results_p2c/';
% posePath = 'data_optitrack/data/ff_return_journey_forward/groundtruth.txt';
tstamp = [];
imgList = [];
ellipse_result1 = [];
ellipse_result2 = [];
fileList = dir(fullfile(ellipsePath, '*.txt'));

for i = 1:length(fileList)
    time = fileList(i).name(1:end-4); % 19
    imgName = [fileList(i).name(1:end-4),'.png']; % 19
    imgName = string(imgName);
    

    two_ellipse_result = readmatrix([ellipsePath, fileList(i).name], 'Delimiter', ' ');
    ellipse1 = 0;
    ellipse2 = 0;

    % 조건 A: 두 개의 행 존재, 조건 B: 모든 행의 두 번째 열이 0이 아님
    if size(two_ellipse_result, 1) == 2 && all(two_ellipse_result(:, 2) ~= 0)
        for j = 1:size(two_ellipse_result, 1)
            if two_ellipse_result(j, 7) == 0
                ellipse1 = two_ellipse_result(j, 1:6);
            elseif two_ellipse_result(j, 7) == 1
                ellipse2 = two_ellipse_result(j, 1:6);
            end
        end
    else
        continue
    end
    % original code
    if ellipse1(1) ~=0 && ellipse2(1) ~= 0
        ellipse_result1 = [ellipse_result1; ellipse1];
        ellipse_result2 = [ellipse_result2; ellipse2];
        tstamp = [tstamp; str2double(time)];
        % imgList = [imgList; imgName];
        imgList = vertcat(imgList, imgName);
    end
end

true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(ellipse_result1,1)
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 0.00001
            P_iss_cam_true(i, :) = true_pose(j, :);
            % 회전행렬 얻기
            R = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            x_axis = -R(:, 1);  % roll (X축 반전)
            y_axis = -R(:, 2);  % pitch (Y축 반전)
            z_axis = R(:, 3);  % yaw 그대로

            % roll <-> yaw 교환 + 부호 반전 적용
            % R_custom = [x_axis,z_axis ,y_axis];  % 최종 순서: Z Y X
            R_custom = [x_axis,z_axis ,y_axis]; 
            R_iss_cam_true{i} = R_custom;


            x = -P_iss_cam_true(i, 2);
            y = -P_iss_cam_true(i, 3);
            z = P_iss_cam_true(i, 4);

            t_iss_cam_true{i} = [x; z; y];

            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
        end
    end
end


% for i = 1:size(ellipse_result1,1)
%     for j = 1:length(true_pose)
%         if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 0.00001
%             P_iss_cam_true(i, :) = true_pose(j, :);
%             % 회전행렬 얻기
%             R = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
%             x_axis = R(:, 1);  % roll (X축 반전)
%             y_axis = R(:, 2);  % pitch (Y축 반전)
%             z_axis = R(:, 3);  % yaw 그대로
% 
%             % roll <-> yaw 교환 + 부호 반전 적용
%             % R_custom = [x_axis,z_axis ,y_axis];  % 최종 순서: Z Y X
%             R_custom = [x_axis,y_axis ,z_axis]; 
%             R_iss_cam_true{i} = R_custom;
% 
% 
%             x = P_iss_cam_true(i, 2);
%             y = P_iss_cam_true(i, 3);
%             z = P_iss_cam_true(i, 4);
% 
%             t_iss_cam_true{i} = [x; z; y];
% 
%             T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
%         end
%     end
% end

tracking_num = size(ellipse_result1, 1);
sample_num = 10;
% 
% figure; hold on; axis equal;
% grid on;
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% for i=1:tracking_num
%     plot_camera_frame(R_iss_cam_true{i}, t_iss_cam_true{i}, 0.5, 'r'); hold on;
%     % pause(0.5);
%     % plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
%     plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
% end
%%Visualize Optimization Result
% figure;
% 
% figure('Name', 'Tracking Optimization + Image View', 'Color', 'w');
% tiledlayout(1, 2); % 1행 2열 layout
% origin = [0 0 0];
% L = 2;
% quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
% quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
% quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
% 
% for i = 1:tracking_num
%     nexttile(1);
%     img = imread(fullfile(imagePath, imgList(i)));
%     imshow(img);
%     title(sprintf("Image %d: %s", i, imgList(i)));
% 
%     % --- 오른쪽: 3D 시각화 ---
%     nexttile(2);
%     % cla; % clear current axes
%     plot_camera_frame(R_iss_cam_true{i}, t_iss_cam_true{i}, 0.5, 'r'); hold on;
%     pause(0.5);
%     plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
%     plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
%     % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
%     grid on; axis equal;
%     xlabel('X'); ylabel('Y'); zlabel('Z');
%     pause(0.5);
% 
%     title("Tracking Optimization");
% end
% set(gcf, 'Color', 'w');
% 
% grid on; axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title("Tracking Optimization");
% hold off;

%%Input Params Setting
% R = [0.5455, 0.145]; % real hatch radius
R = [0.55 0.1365];
% true circle position
pm_position = [10.9349; -10.1; 5.2508];
pm_normal = [0; 1; 0];
T_true_p2c_iss = [eye(3), pm_position; 0 0 0 1];
visualize_colmap = 1; % optional. 

focalLength    = [1338.232 1338.232]; 
principalPoint = [955.4313 727.02765];
imageSize      = [1920 1080];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);


%%P2C RANSAC
PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);
angle_diffs = zeros(1, tracking_num);
figure; 
hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("P2C");
% f=FigureRotator();
for i = 1:tracking_num
    ellipse_params1 = ellipse_result1(i, 2:end);
    ellipse_params2 = ellipse_result2(i, 2:end);
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);
    % fprintf("i=%d, angle_diff=%.2f\n", i, angle_diff{i});
    angle_diffs(i) = angle_diff{i};
    % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 5
        ransac(i) = 0

    end

    z = ( PM_center_cam{i} - Window_center_cam{i} ) / norm(PM_center_cam{i} - Window_center_cam{i});
    x = cross(PM_normal_cam{i}, z);
    x = x/norm(x);
    z = cross(x, PM_normal_cam{i});
    y = PM_normal_cam{i};
    R_cam_p2c = [x, y, z];
    t_cam_p2c = PM_center_cam{i};
    T_cam_p2c{i} = [R_cam_p2c, t_cam_p2c; 0 0 0 1];
    % T_p2c_cam{i} = T_cam_p2c{1} * inv(T_cam_p2c{i});


    z = ( PM_center_cam{i} - Window_center_cam{i});
    z = z / norm(z);
    x = cross(Window_normal_cam{i}, z);
    x = x / norm(x);
    z = cross(x, Window_normal_cam{i});
    y = Window_normal_cam{i};
    R_cam_w2c = [x, y, z];
    t_cam_w2c = Window_center_cam{i};
    T_cam_w2c{i} = [R_cam_w2c, t_cam_w2c; 0 0 0 1];



    plot3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), 'bo', 'MarkerSize', 5); hold on;
    plot3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), 'ro', 'MarkerSize', 5); hold on;

    L = 0.2;
    quiver3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), ...
            PM_normal_cam{i}(1), PM_normal_cam{i}(2), PM_normal_cam{i}(3), ...
            L, 'b', 'LineWidth', 2); hold on;

    quiver3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), ...
            Window_normal_cam{i}(1), Window_normal_cam{i}(2), Window_normal_cam{i}(3), ...
            L, 'r', 'LineWidth', 2); hold on;
    plot_camera_frame( T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 0.5, 'm');
end 




ransac = logical(ransac);
tstamp = tstamp(ransac);
imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
% % T_p2c_cam = T_p2c_cam(ransac);
% T_w2c_cam = T_w2c_cam(ransac);
T_cam_p2c = T_cam_p2c(ransac);
T_cam_w2c = T_cam_w2c(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);

tracking_num = size(R_iss_cam_true, 2);


%%STEP 2 
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



%%Change to global frame 
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


%% 
figure; hold on; axis equal;
grid on; view(45, 45);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);
% Plot the camera
for i=1:tracking_num
    % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3),T_p2c_cam_true{i}(1:3, 4), 0.5, 'b');
    plot_camera_frame( T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'r');
    plot_camera_frame( T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 0.5, 'm');
    plot_camera_frame( T_p2c_cam{best_k}(1:3, 1:3), T_p2c_cam{best_k}(1:3, 4), 0.5, 'k');
    plot_camera_frame( T_cam_p2c{best_k}(1:3, 1:3), T_cam_p2c{best_k}(1:3, 4), 0.5, 'k');

    % pause(0.5);
    plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    % plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
end


%% Corn Visualizatoin with image
% figure;
figure('Name', 'Tracking Optimization + Image View', 'Color', 'w');
tiledlayout(1, 2); % 1행 2열 layout

for i = 1:tracking_num
    nexttile(1);
    img = imread(fullfile(imagePath, imgList(i)));
    imshow(img);
    title(sprintf("Image %d: %s", i, imgList(i)));

    % --- 오른쪽: 3D 시각화 ---
    nexttile(2);
    origin = [0 0 0];
    L = 2;
    quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
    quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
    quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
    % cla; % clear current axes
    % plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);


    plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'c', 'MarkerSize', 0.1); hold on;
    plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'm', 'MarkerSize', 0.1);
    % % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);
    % T_p2c_cam_true_i{i}=inv(T_p2c_cam_true{i})
    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    % plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    % plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우
    plot_camera_frame( T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 0.5, 'm');
    % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    % plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    pause(0.8);

    title("Tracking Optimization");
end
set(gcf, 'Color', 'w');

grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Tracking Optimization");
hold off;



%% DEBUG

figure('Name', 'Tracking Optimization + Image View', 'Color', 'w');
tiledlayout(1, 3); % 1행 2열 layout

for i = 1:tracking_num
    nexttile(1);
    tstampStr = sprintf('%.0f', tstamp(i));
    image = strcat(imagePath, imgList(i, :));
    imshow(image);
    title(sprintf("Image %d: %s", i, imgList(i)));

    % --- 오른쪽: 3D 시각화 ---
    nexttile(2);
    % cla; % clear current axes
    % plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);


    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'c', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'm', 'MarkerSize', 0.1);
    % % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);
    % T_p2c_cam_true_i{i}=inv(T_p2c_cam_true{i})
    view(-10, -45);
    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');

    title("True Pose Reprojection");

    nexttile(3);
    view(-10, -45);
    plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'c', 'MarkerSize', 0.1); hold on;
    plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'm', 'MarkerSize', 0.1);
    plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    for a1 = 1:2
        k1 = ellipses_params{i}(a1, 1); %y
        h = ellipses_params{i}(a1, 2); %x
        a = ellipses_params{i}(a1, 3)/2; %a
        b = ellipses_params{i}(a1, 4)/2; %b
        theta = 90-ellipses_params{i}(a1, 5); %angle (deg)
        t = linspace(0, 2*pi, sample_num);
        x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
        y_ellipse = k1 + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
        x_cam = (x_ellipse - intrinsics.K(1, 3)) / intrinsics.K(1, 1);
        y_cam = (y_ellipse - intrinsics.K(2, 3)) / intrinsics.K(2, 2);
        % Define the rays (from camera center through each ellipse point)
        num_points = length(x_cam);
        rays = zeros(3, num_points);
        scaling_factor = 4.5;
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
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    pause(0.2);

    title("P2C (Corn Ray)");
end
set(gcf, 'Color', 'w');
origin = [0 0 0];
L = 2;
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Tracking Optimization");
hold off;


%% Visualize Optimization Result
% figure;
figure('Name', 'Tracking Optimization + Image View', 'Color', 'w');
tiledlayout(1, 2); % 1행 2열 layout

for i = 1:tracking_num
    nexttile(1);
    img = imread(fullfile(imagePath, imgList(i)));
    imshow(img);
    title(sprintf("Image %d: %s", i, imgList(i)));

    % --- 오른쪽: 3D 시각화 ---
    nexttile(2);
    % cla; % clear current axes
    plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
    plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);


    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'c', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'm', 'MarkerSize', 0.1);
    % % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);
    % T_p2c_cam_true_i{i}=inv(T_p2c_cam_true{i})
    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    pause(0.5);
    
    title("Tracking Optimization");
end
set(gcf, 'Color', 'w');
origin = [0 0 0];
L = 2;
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Tracking Optimization");
hold off;




%% Synthetic
clear; clc; close all;

posePath = 'data/multiview_mixed_data/groundtruth.txt';

%%Step1: Generate Synthetic Circle Data

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
% 
figure; hold on; axis equal;
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
for i=1:tracking_num
    plot_camera_frame(R_iss_cam_true{i}, t_iss_cam_true{i}, 0.5, 'r'); hold on;
    % pause(0.5);
    % plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
end
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

%%Step 2-1: Extract Ellipsoidal Expression
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


PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);

figure; 
hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("P2C");

for i = 1:tracking_num
    ellipse_params1 = ellipse_result1{i};
    ellipse_params2 = ellipse_result2{i};
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);


    % % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 20
        ransac(i) = 0;
 
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
T_cam_p2c = T_cam_p2c(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);

tracking_num = size(R_iss_cam_true, 2);

camera_translation = [10.8951687815517904 -7.8267950003749771 3.4964996589787587];  % Camera position in world coordinates
camera_quaternion = [0.6625124464822644 0.7474290581078724 -0.0045493735392005 -0.0490547097888047];
camera_rotation = quat2rotm(camera_quaternion);        % Camera aligned with world axes (Z-forward, X-right, Y-down)
camera_transformation = [camera_rotation, camera_translation';
                         0, 0, 0, 1];  % Full 4x4 transformation matrix
%%STEP 2 
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







figure;
hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("P2C");

i=9;
plot3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), 'bo', 'MarkerSize', 5); hold on;
plot3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), 'ro', 'MarkerSize', 5); hold on;

L = 0.2;
quiver3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), ...
    PM_normal_cam{i}(1), PM_normal_cam{i}(2), PM_normal_cam{i}(3), ...
    L, 'b', 'LineWidth', 2); hold on;

quiver3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), ...
    Window_normal_cam{i}(1), Window_normal_cam{i}(2), Window_normal_cam{i}(3), ...
    L, 'r', 'LineWidth', 2); hold on;
% Origin
origin = [0 0 0];

% 축 길이
L = 0.2;

% 좌표축 (X: 빨강, Y: 초록, Z: 파랑)
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
T = [ 0.9984  -0.0168  -0.0541   0.0754;
    0.0542   0.0107   0.9985   0.6525;
    -0.0162  -0.9998   0.0116   3.5313;
    0        0        0        1.0000 ];

T_p2c_cam{i} = T * inv(T_cam_p2c{i});
plot_camera_frame( T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'm');
plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'b', 'MarkerSize', 0.1); hold on;
plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'r', 'MarkerSize', 0.1);
%
i=5;

plot3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), 'bo', 'MarkerSize', 5); hold on;
plot3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), 'ro', 'MarkerSize', 5); hold on;

L = 0.2;
quiver3(PM_center_cam{i}(1), PM_center_cam{i}(2), PM_center_cam{i}(3), ...
    PM_normal_cam{i}(1), PM_normal_cam{i}(2), PM_normal_cam{i}(3), ...
    L, 'b', 'LineWidth', 2); hold on;

quiver3(Window_center_cam{i}(1), Window_center_cam{i}(2), Window_center_cam{i}(3), ...
    Window_normal_cam{i}(1), Window_normal_cam{i}(2), Window_normal_cam{i}(3), ...
    L, 'r', 'LineWidth', 2); hold on;
% Origin
origin = [0 0 0];

% 축 길이
L = 0.2;

% 좌표축 (X: 빨강, Y: 초록, Z: 파랑)
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
T = [ 0.9984  -0.0168  -0.0541   0.0754;
    0.0542   0.0107   0.9985   0.6525;
    -0.0162  -0.9998   0.0116   3.5313;
    0        0        0        1.0000 ];

T_p2c_cam{i} = T * inv(T_cam_p2c{i});
plot_camera_frame( T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'm');

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);






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

view(3); % 3D 시점

%%Change to global frame 
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

figure; hold on; axis equal;
grid on; view(45, 45);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% Origin
origin = [0 0 0];

% 축 길이
L = 0.2;

% 좌표축 (X: 빨강, Y: 초록, Z: 파랑)
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);

% Plot the camera
for i=1:tracking_num
    % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3),T_p2c_cam_true{i}(1:3, 4), 0.5, 'b');
    plot_camera_frame( T_p2c_cam{9}(1:3, 1:3), T_p2c_cam{9}(1:3, 4), 0.5, 'r');
    plot_camera_frame( T_cam_p2c{9}(1:3, 1:3), T_cam_p2c{9}(1:3, 4), 0.5, 'm');
    plot_camera_frame( T_cam_p2c{best_k}(1:3, 1:3), T_cam_p2c{best_k}(1:3, 4), 1.5, 'b');
    % pause(0.5);
    plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    % plot_camera_frame(T_iss_cam_true{i}(1:3, 1:3),T_iss_cam_true{i}(1:3, 4), 0.5, 'k');
end

%%Visualize Optimization Result
figure;
% plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
% plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);
% 
% plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
% plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);

for i = 1:tracking_num
    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'b', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'r', 'MarkerSize', 0.1);
    % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);
    plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'k', 'MarkerSize', 1); hold on;
    plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'k', 'MarkerSize', 1);
    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    % plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    % plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    % plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    view(-10, -45);
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    title("Tracking Optimization");
end
% plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'k', 'MarkerSize', 1); hold on;
% plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'k', 'MarkerSize', 1);
set(gcf, 'Color', 'w');
origin = [0 0 0];
L = 2;
quiver3(origin(1), origin(2), origin(3), L, 0, 0, 0, 'r', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, L, 0, 0, 'g', 'LineWidth', 2); hold on;
quiver3(origin(1), origin(2), origin(3), 0, 0, L, 0, 'b', 'LineWidth', 2); hold on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Tracking Optimization");
f=FigureRotator();
k=10;
hold off;



%% Synthetic2
clear; clc; close all;
imagePath = 'data_optitrack/data_synthetic/td_roll_pitch_yaw/gray/';
ellipsePath = 'data_optitrack/data_synthetic/td_roll_pitch_yaw/results_p2c/';
posePath = 'data_optitrack/data_synthetic/td_roll_pitch_yaw/groundtruth.txt';

%%Step1: Generate Synthetic Circle Data

% Define camera intrinsic parameters
focal_length = [1338.232 1338.232];  % Focal lengths (fx, fy)
principal_point = [955.4313 727.02765];  % Principal point (cx, cy)
image_size = [1920 1080];  % Image dimensions (rows, cols)
camera_matrix = [focal_length(1), 0, principal_point(1);
                 0, focal_length(2), principal_point(2);
                 0, 0, 1];

focalLength    = [1338.232 1338.232]; 
principalPoint = [955.4313 727.02765];
imageSize      = [1920 1080];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
R = [0.55 0.1365];
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

for i = 1:length(true_pose)
    P_iss_cam_true(i, :) = true_pose(i, :);
    R_iss_cam_temp = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);

    % Z축 기준 시계 방향 180도 회전 행렬 변환 추가
    R_z = [
        -1,  0,  0;
        0, -1,  0;
        0,  0,  1;
        ];

    % y축 기준 시계 방향 30도 회전 행렬 변환 추가
    R_y = [
        0.866,  0,  0.5;
        0, 1,  0;
        -0.5,  0,  0.866;
        ];



    % 기존 회전 행렬에 Z축 회전 적용
    R_iss_cam_true{i} = R_iss_cam_temp;

    % t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
    x = P_iss_cam_true(i, 2);
    y = P_iss_cam_true(i, 3);
    z = P_iss_cam_true(i, 4);

    t_iss_cam_true{i} = [x; z; y];

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

%%Step 2-1: Extract Ellipsoidal Expression
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


    % % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 20
        ransac(i) = 0;
 
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
T_cam_p2c = T_cam_p2c(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);

tracking_num = size(R_iss_cam_true, 2);

camera_translation = [10.8951687815517904 -7.8267950003749771 3.4964996589787587];  % Camera position in world coordinates
camera_quaternion = [0.6625124464822644 0.7474290581078724 -0.0045493735392005 -0.0490547097888047];
camera_rotation = quat2rotm(camera_quaternion);        % Camera aligned with world axes (Z-forward, X-right, Y-down)
camera_transformation = [camera_rotation, camera_translation';
                         0, 0, 0, 1];  % Full 4x4 transformation matrix
%%STEP 2 
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

view(3); % 3D 시점

%%Change to global frame 
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

%%Visualize Optimization Result
figure;
% plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
% plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.5); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.5);

for i = 1:tracking_num
    plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'b', 'MarkerSize', 0.1); hold on;
    plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'r', 'MarkerSize', 0.1);
    % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);

    true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

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


%%



%% Function

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