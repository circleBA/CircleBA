clc;
close all;
clear variables; %clear classes;

%% preprocessing - only detected ellipse


imagePath = 'data/ff_return_journey_forward/gray/';
ellipsePath = 'data/ff_return_journey_forward/results_p2cc/';
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
posePath = 'data/ff_return_journey_forward/groundtruth.txt';
droid_path = 'data/ff_return_journey_forward/DROID_P2C_ff_return_journey_forward.txt';
orb_path = 'data/ff_return_journey_forward/ORB_P2C_ff_return_journey_forward.txt';



tstamp = [];
imgList = [];
ellipse_result1 = [];
ellipse_result2 = [];
fileList = dir(fullfile(ellipsePath, '*.txt'));
for i = 1:length(fileList)
    time = fileList(i).name(1:19);
    imgName = [fileList(i).name(1:19),'.png'];

    two_ellipse_result = readmatrix([ellipsePath, fileList(i).name], 'Delimiter', ' ');
    ellipse1 = 0;
    ellipse2 = 0;
    for j = 1:size(two_ellipse_result,1)
        if two_ellipse_result(j, 2) ~= 0
            if two_ellipse_result(j, 7) == 0
                ellipse1 = two_ellipse_result(j, 1:6);
            end
        end

        if two_ellipse_result(j, 2) ~= 0
            if two_ellipse_result(j, 7) == 1 
                ellipse2 = two_ellipse_result(j, 1:6);
            end
        end
    end

    if ellipse1(1) ~=0 && ellipse2(1) ~=0
        ellipse_result1 = [ellipse_result1; ellipse1];
        ellipse_result2 = [ellipse_result2; ellipse2];
        tstamp = [tstamp; str2double(time)];
        imgList = [imgList; imgName];
    end
end

%%
true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(ellipse_result1,1)
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 100000
            P_iss_cam_true(i, :) = true_pose(j, :);
            R_iss_cam_true{i} = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
        end
    end
end

tracking_num = size(ellipse_result1, 1);
sample_num = 1000;

%% Input Params Setting
R = [0.5455, 0.145]; % real hatch radius
% R = [0.555 0.141];
% true circle position
pm_position = [10.9349; -10.1; 5.2508];
pm_normal = [0; 1; 0];
T_true_p2c_iss = [eye(3), pm_position; 0 0 0 1];
visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

%% P2C RANSAC
PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);

for i = 1:tracking_num
    ellipse_params1 = ellipse_result1(i, 2:end);
    ellipse_params2 = ellipse_result2(i, 2:end);
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);

    % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 2000
        ransac(i) = 0;
    else
        % if i == 1 
        %     tstampStr = sprintf('%.0f', tstamp(i));
        %     image = strcat(imagePath, imgList(i, :));
        %     figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
        %     % pause(4);
        % end
        % tstampStr = sprintf('%.0f', tstamp(i));
        % image = strcat(imagePath, imgList(i, :));
        % figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
        % % pause(0.5);
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
    
end
% for i=1:tracking_num
%     T_p2c_cam{i} = T_cam_p2c{7} * inv(T_cam_p2c{i});
% end

% figure;
% for i = 1:tracking_num
% 
%     plot_camera_frame(T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 0.5, 'r');
%     plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'g');
% 
%     view(-10, -45);
%     % view(0, 0);
%     grid on; axis equal;
%     xlabel('X'); ylabel('Y'); zlabel('Z');
% 
%     title("Tracking Optimization");
% end

ransac = logical(ransac);
tstamp = tstamp(ransac);
imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
T_p2c_cam = T_p2c_cam(ransac);
T_w2c_cam = T_w2c_cam(ransac);
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

% T_p2c_iss = T_p2c_cam{7} * inv(T_iss_cam_true{7});
% for i = 1:tracking_num
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end
% T_p2c_iss = T_p2c_cam{1} * inv(T_iss_cam_true{1});
% for i = 1:tracking_num
%     T_p2c_iss = T_p2c_cam{i} * inv(T_iss_cam_true{i});
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end




%% Choose Initial Map Points with Cone Equation
% Visualize the cone
z_means = zeros(1, tracking_num);
InitialMap_PM_all = cell(1, tracking_num);
InitialMap_Win_all = cell(1, tracking_num);
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
        % % Plot the rays
        % for j = 1:40:num_points
        %     plot3([0 rays(1, j)], ...
        %         [0 rays(2, j)], ...
        %         [0 rays(3, j)], 'g-');
        % end
        % % Plot the ellipse points on the image plane (z = 1)
        % plot3(x_cam, y_cam, ones(1, num_points), 'mo');
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
    % plot3(PM_center_cam{1}(1, 1) + PM_circle_3D(1, :), PM_center_cam{1}(2, 1) + PM_circle_3D(2, :), PM_center_cam{1}(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;

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
    % plot3(Window_center_cam{1}(1, 1) + Window_circle_3D(1, :), Window_center_cam{1}(2, 1) + Window_circle_3D(2, :), Window_center_cam{1}(3, 1) + Window_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;
    InitialMap_PM_all{k} = [PM_center_cam{k}(1) + PM_circle_3D(1, :); ...
                            PM_center_cam{k}(2) + PM_circle_3D(2, :); ...
                            PM_center_cam{k}(3) + PM_circle_3D(3, :)];

    InitialMap_Win_all{k} = [Window_center_cam{k}(1) + Window_circle_3D(1, :); ...
                             Window_center_cam{k}(2) + Window_circle_3D(2, :); ...
                             Window_center_cam{k}(3) + Window_circle_3D(3, :)];
    % axis equal;
    % grid on;
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % ylim([-10 10]);
    % zlim([-10 10]);
    % xlim([-10 10]);
    % title('3D Visualization of Cone in Camera Frame');
    % hold off;
    % f = FigureRotator();
    z_diff = (Window_center_cam{k}(3,1) + Window_circle_3D(3,:)) - ...
         (PM_center_cam{k}(3,1) + PM_circle_3D(3,:));

    z_means(k) = mean(z_diff);  

end
[~, best_k] = min(z_means);
disp(best_k);
InitialMap_PM = InitialMap_PM_all{best_k};
InitialMap_Win = InitialMap_Win_all{best_k};

%% Change to global frame 
for i=1:tracking_num
    T_p2c_cam{i} = T_cam_p2c{best_k} * inv(T_cam_p2c{i});
end
T_p2c_iss = T_p2c_cam{best_k} * inv(T_iss_cam_true{best_k});
for i = 1:tracking_num
    T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
end


%% Tracking Optimization
disp("optimization");
ransac2 = ones(1, tracking_num);
EstimatedPose{best_k} = T_p2c_cam{best_k};
% EstimatedPose{1} = T_p2c_cam{1};
PM_normal_i = EstimatedPose{best_k} * [PM_normal_cam{best_k}; 1];
% PM_normal_i = EstimatedPose{1} * [PM_normal_cam{1}; 1];
PM_normal_init = PM_normal_i(1:3)/norm(PM_normal_i(1:3));
% figure;
% figure;
% plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'g');
% plot_camera_frame(T_p2c_cam{best_k}(1:3, 1:3), T_p2c_cam{best_k}(1:3, 4), 0.5, 'r');
% plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
% view(-10, -45);
% % view(0, 0);
% grid on; axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title("Tracking Optimization");
% f=FigureRotator();


for i =1:tracking_num
    if i==best_k
        continue
    end

    T_relative = inv( inv(T_p2c_cam{best_k}) * T_p2c_cam{i} );
    % T_relative = eye(4);
    % T_relative = inv(T_p2c_cam{best_k}) * T_p2c_cam{i} ;
    xi_init = logSE3(T_relative);

    objective_function = @(params) EReprojection_motiononly3( ...
        params, InitialMap_PM, InitialMap_Win, C_pm{i}, C_win{i}, intrinsics.K);
    
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
                           'Display','iter','FiniteDifferenceType','forward');
    
    [opt_params, resnorm] = lsqnonlin(objective_function, xi_init, [], [], options);
    
    Relative_Trans_opt = expSE3(opt_params);
    opt_params_store{i} = opt_params;
    drawing{i}=expSE3(opt_params_store{i});
    R_relative_opt = Relative_Trans_opt(1:3, 1:3);
    T_relative_opt = Relative_Trans_opt(1:3, 4);
    
    EstimatedPose{i} = T_p2c_cam{best_k} * inv(Relative_Trans_opt);

    PM_normal = EstimatedPose{i} * [PM_normal_cam{i}; 1];
    PM_normal_esti = PM_normal(1:3) / norm(PM_normal(1:3));
    angle_diff2{i} = acos(dot(PM_normal_init, PM_normal_esti)) * 180 / pi;
    if angle_diff2{i} > 300
        ransac2(i) = 0;
    end
    for j = 1:sample_num
        P_camera = R_relative_opt * InitialMap_PM(:, j) + T_relative_opt;
        % Convert to homogeneous coordinates by appending a row of ones
        P_camera_homogeneous = P_camera;

        % Project 3D points to 2D using intrinsic matrix
        projected_points = intrinsics.K * P_camera_homogeneous;

        % Convert from homogeneous to 2D coordinates
        projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
        projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);

        % Extract the 2D points
        u1(j) = projected_points(1, :);
        v1(j) = projected_points(2, :);

        e1(j) = abs([u1(j) v1(j) 1]* C_pm{i} * [u1(j); v1(j); 1]);
    end
       

    for j = 1:sample_num
        P_camera = R_relative_opt * InitialMap_Win(:, j) + T_relative_opt;

        % Convert to homogeneous coordinates by appending a row of ones
        P_camera_homogeneous = P_camera;

        % Project 3D points to 2D using intrinsic matrix
        projected_points = intrinsics.K * P_camera_homogeneous;

        % Convert from homogeneous to 2D coordinates
        projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
        projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);

        % Extract the 2D points
        u2(j) = projected_points(1, :);
        v2(j) = projected_points(2, :);

        e2(j) = abs([u2(j) v2(j) 1]* C_win{i} * [u2(j); v2(j); 1]);
    end
    % tstampStr = sprintf('%.0f', tstamp(i),i);
    % image = strcat(imagePath, imgList(i, :));
    % imshow(image); hold on;
    % % plot(u1_orgin, v1_origin, 'magenta', 'MarkerSize',5, 'LineWidth', 4); hold on;% Plot reprojected points in red
    % plot(u1, v1, 'b', 'MarkerSize',5, 'LineWidth', 4); hold on;% Plot reprojected points in red
    % plot(u2, v2, 'r', 'MarkerSize',5, 'LineWidth', 4); % Plot reprojected points in red
    % title(sprintf('Frame %d - Timestamp: %s', i, tstampStr));
    % pause(0.5);

end
ransac2 = logical(ransac2);
tstamp = tstamp(ransac2);
imgList = imgList(ransac2, :);
R_iss_cam_true = R_iss_cam_true(ransac2);
t_iss_cam_true = t_iss_cam_true(ransac2);
T_iss_cam_true = T_iss_cam_true(ransac2);
T_p2c_cam = T_p2c_cam(ransac2);
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



%% Visualize Optimization Result
figure;
% plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
% plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 4); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 4);

for i = 1:tracking_num
    if i == best_k
        continue;
    end
    % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'g');
    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우
    R_relative_opt = drawing{i}(1:3, 1:3);
    T_relative_opt = drawing{i}(1:3, 4);
    % plot_camera_frame(R_relative_opt, T_relative_opt, 0.5, 'b'); hold on;
    R = T_relative(1:3, 1:3);
    t = T_relative(1:3, 4);
    % plot_camera_frame(R, t, 0.5, 'b'); hold on;
    %
    % 
    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
    % % 
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    % plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'r');

    view(-10, -45);
    % view(0, 0);
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    title("Tracking Optimization");

    % if i == 1
    %     plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    %     plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
    %     % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    %     view(-10, -45); 
    %     % view(0, 0); 
    %     grid on; axis equal;
    %     xlabel('X'); ylabel('Y'); zlabel('Z');
    %     title("Tracking Optimization");
    %     % pause(4);
    % end
    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
    % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    view(-10, -45); 
    % view(0, 0); 
    % plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 0.5, 'm');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("Tracking Optimization");
    % pause(1);
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








%% Evaluation

% [RPE_RMSE_BA,RPE_BA] = calcRPE(BAEstimatedPose, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_BA,ATE_BA] = calcATE(BAEstimatedPose, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_BA,RMD_BA] = calcRMD(BAEstimatedPose, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_MO,RPE_MO] = calcRPE(EstimatedPose, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_MO,ATE_MO] = calcATE(EstimatedPose, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_MO,RMD_MO] = calcRMD(EstimatedPose, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_p2c,RPE_p2c] = calcRPE(T_p2c_cam, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_p2c,ATE_p2c] = calcATE(T_p2c_cam, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_p2c,RMD_p2c] = calcRMD(T_p2c_cam, T_p2c_cam_true, 'RMSE');


[RMD_RMSE_p2cpc,RMD_p2cpc] = calcRMD(T_p2c_cam, T_w2c_cam_true, 'RMSE');

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
fprintf("p2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2c, RPE_RMSE_p2c, RMD_RMSE_p2c);
% fprintf("DROID result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_droid, RPE_RMSE_droid, RMD_RMSE_droid);
% fprintf("COLMAP result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_colmap, RPE_RMSE_colmap, RMD_RMSE_colmap);
fprintf("reprojection error result \n PM: %f \n Win %f \n total %f \n", rms(e1), rms(e2), reprojection_error_total);




%%
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

    % % ellipse center point euclidian distance
    % x1 = ellipse_param1(2);
    % y1 = ellipse_param1(1);
    % first_center = [x1; y1; 1];
    % x2 = ellipse_param2(2);
    % y2 = ellipse_param2(1);
    % next_center = [x2; y2; 1];
    Qpm = Qpm / norm(Qpm, 'fro');  % 또는 Qpm = Qpm / Qpm(3,3);
    Qwin = Qwin / norm(Qwin, 'fro');



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
        %residuals(i) = 1000*val_pm + 1000*val_win;
        %residuals(i) = 1000*(abs(val_pm) + abs(val_win));
        residuals(i) = sqrt(1000*(val_pm)^2 + 0*(val_win)^2);

        
        %fprintf("i=%d, residual = %.10f\n", i, residuals(i));
    end
    

    % If you also have constraints in the first image or other constraints,
    % you can stack them in the residual vector as well.

end
function F = EReprojection_motiononly(params, PM_Point3D, Win_Point3D, Qpm, Qwin, K)
    % roll = params(1);
    % pitch = params(2);
    % yaw = params(3);
    % Rk = angle2dcm( yaw, pitch, roll );
    % Tk = params(4:6)';
    % T = expSE3(params);
    % n = size(PM_Point3D, 2);


    % F1 = zeros(n, 1);
    % F2 = zeros(n, 1);
    T = expSE3(params);
    N = size(PM_Point3D, 2);

    residuals = zeros(N,1);
    for i = 1:N
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

        % 5. That val is the i-th residual
        residuals(i) = val_pm + val_win;
    end
end

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