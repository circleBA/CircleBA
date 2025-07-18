clc;
close all;
clear variables; %clear classes;

%% preprocessing - only detected ellipse

% imagePath = 'data_optitrack/data/sfm_test/gray/';
% ellipsePath = 'data_optitrack/data/sfm_test/results_p2c/';
% posePath = 'data_optitrack/data/sfm_test/groundtruth.txt';

imagePath = 'data_optitrack/data/sfm/gray/';
ellipsePath = 'data_optitrack/data/sfm/results_p2c/';
posePath = 'data_optitrack/data/sfm/groundtruth.txt';

% imagePath = 'data_optitrack/data/td_roll_pitch_yaw/gray/';
% ellipsePath = 'data_optitrack/data/td_roll_pitch_yaw/results_p2c/';
% posePath = 'data_optitrack/data/td_roll_pitch_yaw/groundtruth.txt';

tstamp = [];
imgList = [];
ellipse_result1 = [];
ellipse_result2 = [];
fileList = dir(fullfile(ellipsePath, '*.txt'));
for i = 1:length(fileList)
    time = fileList(i).name(1:end-4); % 19
    imgName = [fileList(i).name(1:end-4),'.png']; % 19
    imgName = string(imgName);
    
    % time과 imgName 출력
    % disp(['Time: ', time]);
    % disp(['Image Name: ', imgName]);

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


%%

true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(ellipse_result1,1)
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 0.00001
            P_iss_cam_true(i, :) = true_pose(j, :);
            % 회전행렬 얻기
            R = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            x_axis = R(:, 1);  % roll (X축 반전)
            y_axis = R(:, 2);  % pitch (Y축 반전)
            z_axis = R(:, 3);  % yaw 그대로

            % roll <-> yaw 교환 + 부호 반전 적용
            R_custom = [x_axis,y_axis ,z_axis];  % 최종 순서: Z Y X
            R_iss_cam_true{i} = R_custom;


            x = P_iss_cam_true(i, 2);
            y = P_iss_cam_true(i, 3);
            z = P_iss_cam_true(i, 4);

            t_iss_cam_true{i} = [x; z; y];

            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
        end
    end
end
% true_pose = readmatrix(posePath, 'Delimiter', ' ');
% for i = 1:size(ellipse_result1,1)
%     for j = 1:length(true_pose)
%         if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 0.00001
%             P_iss_cam_true(i, :) = true_pose(j, :);
%             R_iss_cam_temp = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
% 
% 
% 
% 
% 
%             % 기존 회전 행렬에 Z축 회전 적용
%             R_iss_cam_true{i} = R_iss_cam_temp;
% 
%             % t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
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




%% Input Params Setting
% R = [0.5455, 0.145]; % real hatch radius
R = [0.55 0.1365];
% true circle position
% pm_position = [10.9349; -10.1; 5.2508];
% pm_normal = [0; 1; 0];
% T_true_p2c_iss = [eye(3), pm_position; 0 0 0 1];
visualize_colmap = 1; % optional. 

focalLength    = [1338.232 1338.232]; 
principalPoint = [955.4313 727.02765];
imageSize      = [1920 1080];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);


%% Ellipse Detection 
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);
angle_diffs = zeros(1, tracking_num);

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

end
ransac = logical(ransac);
tstamp = tstamp(ransac);
imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);

ellipses_params = ellipses_params(ransac);


tracking_num = size(R_iss_cam_true, 2);

%% 4. Pattern Localization 
figure; 
hold on;
z_means = zeros(1, tracking_num);
Q_PM=cell(1,tracking_num);
Q_Win=cell(1,tracking_num);


for k = 1:tracking_num
    for i = 1:2
        k1 = ellipses_params{k}(i, 1); % y
        h  = ellipses_params{k}(i, 2); % x
        a  = ellipses_params{k}(i, 3) / 2;
        b  = ellipses_params{k}(i, 4) / 2;
        theta = 90 - ellipses_params{k}(i, 5); % degrees

        % 중심 및 장단축 벡터
        center = [h; k1];
        e0 = a * [cosd(theta); sind(theta)];
        e1 = b * [-sind(theta); cosd(theta)];

        % 카메라 intrinsics
        fx = intrinsics.FocalLength(1);
        fy = intrinsics.FocalLength(2);
        cx = intrinsics.PrincipalPoint(1);
        cy = intrinsics.PrincipalPoint(2);

        % ellipse의 vertex 4개
        a0 = [h + e0(1); k1 + e0(2)];
        a1 = [h - e0(1); k1 - e0(2)];
        b0 = [h + e1(1); k1 + e1(2)];
        b1 = [h - e1(1); k1 - e1(2)];

        % canonical coords (normalized image coordinates)
        a0_bar = [(a0(1) - cx) / fx; (a0(2) - cy) / fy];
        a1_bar = [(a1(1) - cx) / fx; (a1(2) - cy) / fy];
        b0_bar = [(b0(1) - cx) / fx; (b0(2) - cy) / fy];
        b1_bar = [(b1(1) - cx) / fx; (b1(2) - cy) / fy];

        % canonical 중심과 semiaxes
        e0_bar = (a0_bar - a1_bar) / 2;
        e1_bar = (b0_bar - b1_bar) / 2;
        uc_bar = (a0_bar(1) + a1_bar(1) + b0_bar(1) + b1_bar(1)) / 4;
        vc_bar = (a0_bar(2) + a1_bar(2) + b0_bar(2) + b1_bar(2)) / 4;

        % Q 계산
        e0u = e0_bar(1); e0v = e0_bar(2);
        e1u = e1_bar(1); e1v = e1_bar(2);
        q_a = e0u^2 + e1u^2;
        q_b = e0u*e0v + e1u*e1v;
        q_c = e0v^2 + e1v^2;
        q_d = -uc_bar * q_a - vc_bar * q_b;
        q_e = -uc_bar * q_b - vc_bar * q_c;
        q_f = q_a * uc_bar^2 + q_c * vc_bar^2 + 2 * q_b * uc_bar * vc_bar - 1;

        Q = [q_a, q_b, q_d;
             q_b, q_c, q_e;
             q_d, q_e, q_f];

        if i == 1
            Q_PM{k} = Q;
        else
            Q_Win{k} = Q;
        end
    end
end

%% Pattern Position
PatternPose_PM = cell(1, tracking_num);
PatternPose_Win = cell(1, tracking_num);

pattern_diameter = [0.55 0.1365];

for k = 1:tracking_num
    Q = Q_PM{k};  % 또는 Q_Win{k}

    % Eigenvalue decomposition
    [V, D] = eig(Q);
    lambda = diag(D);
    [lambda_sorted, idx] = sort(lambda, 'descend'); % λ0 ≥ λ1 > 0 > λ2

    q0 = V(:, idx(1));
    q1 = V(:, idx(2));  % 안 쓰임
    q2 = V(:, idx(3));

    lambda0 = lambda_sorted(1);
    lambda1 = lambda_sorted(2);
    lambda2 = lambda_sorted(3);

    % Avoid invalid or near-zero denominators
    if lambda1 - lambda2 == 0 || lambda0 - lambda2 == 0 || lambda0 * lambda2 >= 0
        warning("Frame %d has invalid eigenvalue configuration.", k);
        continue;
    end

    % Compute x_c
    scale = (pattern_diameter(1) / (-lambda0 * lambda2)) * ...
            ((lambda0 - lambda1) / (lambda1 - lambda2));
    xc = sqrt(scale) * (lambda2 * q0 + lambda0 * q2);

    % Sign disambiguation (ensure x > 0 → in front of camera)
    if xc(1) < 0
        xc = -xc;
    end

    PatternPose_PM{k} = xc;
end

for k = 1:tracking_num
    Q = Q_Win{k};  % 또는 Q_Win{k}

    % Eigenvalue decomposition
    [V, D] = eig(Q);
    lambda = diag(D);
    [lambda_sorted, idx] = sort(lambda, 'descend'); % λ0 ≥ λ1 > 0 > λ2

    q0 = V(:, idx(1));
    q1 = V(:, idx(2));  % 안 쓰임
    q2 = V(:, idx(3));

    lambda0 = lambda_sorted(1);
    lambda1 = lambda_sorted(2);
    lambda2 = lambda_sorted(3);

    % Avoid invalid or near-zero denominators
    if lambda1 - lambda2 == 0 || lambda0 - lambda2 == 0 || lambda0 * lambda2 >= 0
        warning("Frame %d has invalid eigenvalue configuration.", k);
        continue;
    end

    % Compute x_c
    scale = (pattern_diameter(2) / (-lambda0 * lambda2)) * ...
            ((lambda0 - lambda1) / (lambda1 - lambda2));
    xc = sqrt(scale) * (lambda2 * q0 + lambda0 * q2);

    % Sign disambiguation (ensure x > 0 → in front of camera)
    if xc(1) < 0
        xc = -xc;
    end

    PatternPose_Win{k} = xc;
end

%% Transformation to the Global Coordinates
% Global Coordinate Frame -3D case
PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
ransac = ones(1, tracking_num);
angle_diffs = zeros(1, tracking_num);

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

    y = ( PatternPose_PM{i} - PatternPose_Win{i} ) / norm(PatternPose_PM{i} - PatternPose_Win{i});
    x = cross(PM_normal_cam{i}, y);
    x = x/norm(x);
    y = cross(x, PM_normal_cam{i});
    z = PM_normal_cam{i};
    R_cam_p2c = [x, y, z];
    t_cam_p2c = PatternPose_PM{i};
    T_cam_p2c{i} = [R_cam_p2c, t_cam_p2c; 0 0 0 1];
    % T_p2c_cam{i} = T_cam_p2c{1} * inv(T_cam_p2c{i});


    y = ( PM_center_cam{i} - Window_center_cam{i});
    z = z / norm(z);
    x = cross(Window_normal_cam{i}, z);
    x = x / norm(x);
    z = cross(x, Window_normal_cam{i});
    y = Window_normal_cam{i};
    R_cam_w2c = [x, y, z];
    t_cam_w2c = Window_center_cam{i};
    T_cam_w2c{i} = [R_cam_w2c, t_cam_w2c; 0 0 0 1];

end
figure;
histogram(angle_diffs, 50);  % bin 개수는 필요에 따라 조절
xlabel('angle diff');
ylabel('Count');
title('Distribution of angle difference between PM and Window');
grid on;
fprintf('Angle diff statistics:\n');
fprintf('  Mean  = %.2f\n', mean(angle_diffs));
fprintf('  Std   = %.2f\n', std(angle_diffs));
fprintf('  Min   = %.2f\n', min(angle_diffs));
fprintf('  Max   = %.2f\n', max(angle_diffs));

%
% for i = 1:length(tstamp)
%     tstampStr = sprintf('%.4f', tstamp(i));  % 예: '10.2000'
%     % 점을 파일명에 쓸 수 없으므로 점을 밑줄 등으로 대체
%     tstampStr = strrep(tstampStr, '.', '');  % 예: '10_2000'
%     image = strcat(imagePath, imgList(i, :));
%     fig = figure('Visible', 'off');
%     figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
%     saveas(fig, fullfile('data_optitrack/data/ff_return_journey_left/ellipse_vis/', ['ellipse_' tstampStr '.png']));
%     close(fig);
% end




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
    % plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'r', 'MarkerSize', 0.1);


    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'c', 'MarkerSize', 0.1); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'm', 'MarkerSize', 0.1);
    % % transformed_points = EstimatedPose{i}(1:3, 1:3) * InitialMap_PM_all{i} + EstimatedPose{i}(1:3, 4);
    % transformed_points_win = EstimatedPose{i}(1:3, 1:3) * InitialMap_Win_all{i} + EstimatedPose{i}(1:3, 4);
    % plot3(transformed_points(1, :), transformed_points(2, :), transformed_points(3, :), 'c', 'MarkerSize', 0.1);
    % % plot3(transformed_points_win(1, :), transformed_points_win(2, :), transformed_points_win(3, :), 'm', 'MarkerSize', 0.1);
    % T_p2c_cam_true_i{i}=inv(T_p2c_cam_true{i})
    % true_points=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_PM_all{i}+T_p2c_cam_true{i}(1:3, 4);
    % true_points_win=T_p2c_cam_true{i}(1:3, 1:3)*InitialMap_Win_all{i}+T_p2c_cam_true{i}(1:3, 4);
    % plot3(true_points(1, :), true_points(2, :),true_points(3, :), 'Color',[0 0.4470 0.7410], 'MarkerSize', 0.1);
    % plot3(true_points_win(1, :), true_points_win(2, :), true_points_win(3, :), 'Color',[0.8500 0.3250 0.0980], 'MarkerSize', 0.1);

    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :),  'bo', 'MarkerSize', 4); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :),  'ro', 'MarkerSize', 4);

    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;     % Relative_Trans_opt = expSE3(opt_params_store{i});  % opt_params를 미리 저장해둔 경우

    plot_camera_frame(T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 0.5, 'k'); hold on;
    % plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    % plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'g');
    % plot_camera_frame(T_rel(1:3, 1:3), T_rel(1:3, 4), 0.5, 'k');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    pause(0.7);
    
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





%% Initial Map Visualize the cone
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
            residuals = [residuals; val_pm;val_win];
        end
    end

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

