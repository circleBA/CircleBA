clc;
close all;
clear variables; %clear classes;

%% Sanity Check 

imagePath = 'data/td_yaw4/gray_aamed/';
ellipsePath = 'data/td_yaw4/results_p2c_rect.txt';
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
posePath = 'data/td_yaw4/groundtruth.txt';

two_ellipse_result = readmatrix(ellipsePath, 'Delimiter', ' ');
tstamp = [];
ellipse_result1 = [];
ellipse_result2 = [];
for i = 1:length(two_ellipse_result)/2
    if two_ellipse_result(i*2, 2) ~= 0 && two_ellipse_result(i*2-1, 2) ~= 0
        tstamp = [tstamp, two_ellipse_result(i*2, 1)];
        ellipse_result1 = [ellipse_result1; two_ellipse_result(i*2-1, 1:6)];
        ellipse_result2 = [ellipse_result2; two_ellipse_result(i*2, 1:6)];
    end
end


true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:length(ellipse_result1)
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 10
            P_iss_cam(i, :) = true_pose(j, :);
        end
    end
end

tracking_num = length(ellipse_result1);
sample_num = 100;
%% Input Params Setting
R = [0.55, 0.145]; % real hatch radius
% true circle position
pm_position = [10.9349; -10.1; 5.2508];
pm_normal = [0; 1; 0];
T_true_p2c_iss = [eye(3), pm_position; 0 0 0 1];
visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

%% Check Detected Ellipses
PM_normal_cam = cell(1, tracking_num);
PM_center_cam = cell(1, tracking_num);
Window_normal_cam = cell(1, tracking_num);
Window_center_cam = cell(1, tracking_num);
C_pm = cell(1, tracking_num);
C_win = cell(1, tracking_num);
ellipses_params = cell(1, tracking_num);
for i = 1:tracking_num
    timestamp(i) = ellipse_result1(i, 1);
    ellipse_params1 = ellipse_result1(i, 2:end);
    ellipse_params2 = ellipse_result2(i, 2:end);
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    tstampStr = sprintf('%.0f', timestamp(i));
    image = strcat(imagePath, tstampStr, '.png');
    figure_detected_two_ellipse(image, ellipses_params, tstampStr);
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);
    pause(1);
    R_iss_cam{i} = quat2rotm([P_iss_cam(i, 8), P_iss_cam(i, 5:7)]);
    t_iss_cam{i} = P_iss_cam(i, 2:4)';
    T_iss_cam{i} = [R_iss_cam{i}, t_iss_cam{i}; 0 0 0 1];

    z = ( PM_center_cam{i} - Window_center_cam{i} ) / norm(PM_center_cam{i} - Window_center_cam{i});
    x = cross(PM_normal_cam{i}, z);
    x = x/norm(x);
    z = cross(x, PM_normal_cam{i});
    y = PM_normal_cam{i};
    R_p2c_cam = [x, y, z];
    t_p2c_cam = PM_center_cam{i};
    T_p2c_cam{i} = [R_p2c_cam, t_p2c_cam; 0 0 0 1];
    T_cam_world{i} = T_true_p2c_iss * inv(T_p2c_cam{i});
    T_cam_p2c{i} = T_p2c_cam{1} * inv(T_p2c_cam{i});
end
T_iss_cam{1} = eye(4);

%% Cone Equation
% Visualize the cone
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

InitialMap_PM = [PM_center_cam{1}(1, 1) + PM_circle_3D(1, :); PM_center_cam{1}(2, 1) + PM_circle_3D(2, :); PM_center_cam{1}(3, 1) + PM_circle_3D(3, :)];
InitialMap_Win = [Window_center_cam{1}(1, 1) + Window_circle_3D(1, :); Window_center_cam{1}(2, 1) + Window_circle_3D(2, :); Window_center_cam{1}(3, 1) + Window_circle_3D(3, :)];



%% Optimization
disp("optimization");
EstimatedPose = cell(1, tracking_num);
EstimatedPose_camframe = cell(1, tracking_num);
EstimatedPose{1} = T_iss_cam{1}; %T_cam_world{1};
EstimatedPose_camframe{1} = T_cam_p2c{1};
figure;
for i = 2:tracking_num
    % R_init = eye(3);
    % T_init = [0;0;0];
    % R_init = R_true*[0.99 0 0; 0 0.99 0; 0 0 0.99];
    % T_init = T_true+[0.1; -0.1; -0.1];
    T_relative = inv( inv(T_cam_world{1}) * T_cam_world{i} );
    R_init = T_relative(1:3, 1:3);
    T_init = T_relative(1:3, 4);
    %optimization per frame
    objective_function = @(params) EReprojection_motiononly(params, InitialMap_PM, InitialMap_Win, C_pm{i}, C_win{i}, intrinsics.K);
    initial_params = [R_init, T_init];
    
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
    [opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);
    
    R_relative_opt= opt_params(1:3, 1:3);
    T_relative_opt = opt_params(1:3, 4);
    Relative_Trans_opt = [R_relative_opt, T_relative_opt; 0 0 0 1];
    
    EstimatedPose{i} = T_cam_world{1} * Relative_Trans_opt;
    EstimatedPose_camframe{i} = T_cam_p2c{1} * Relative_Trans_opt;
    
    %visualize reprojection with optimized R,t
    for j = 1:sample_num
        P_camera = R_relative_opt * InitialMap_Win(:, j) + T_relative_opt;
        disp(T_relative_opt);
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
        P_camera = R_relative_opt * InitialMap_PM(:, j) + T_relative_opt;

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

    tstampStr = sprintf('%.0f', timestamp(i));
    image = strcat(imagePath, tstampStr, '.png');
    imshow(image); hold on;
    plot(u1, v1, 'co', 'MarkerSize',1); hold on;% Plot reprojected points in red
    plot(u2, v2, 'mo', 'MarkerSize',1); % Plot reprojected points in red

    pause(0.5);
end

%% Full bundle adjustment
poses = [];
for i = 2:tracking_num
    T_relatives = inv( inv(EstimatedPose_camframe{1}) * EstimatedPose_camframe{i} );
    R_init = T_relatives(1:3, 1:3);
    T_init = T_relatives(1:3, 4);
    init = [R_init, T_init];
    poses = [poses, init];
end

%optimization per frame
objective_function = @(params) EReprojection_localmapping(params, tracking_num, sample_num, C_pm, C_win, intrinsics.K);
initial_params = [poses, InitialMap_PM, InitialMap_Win];

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
[opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);

for i = 1:tracking_num-1
    T_relative_opt = opt_params(1:3, 4*i-3:4*i);
    Relative_Trans_opt = [T_relative_opt; 0 0 0 1];
    BAEstimatedPose{i+1} = EstimatedPose_camframe{1} * Relative_Trans_opt;
end
BAEstimatedPose{1} = EstimatedPose{1};
PM_Map_ba = opt_params(1:3, 4*(tracking_num-1)+1:4*(tracking_num-1)+sample_num);
Win_Map_ba = opt_params(1:3, 4*(tracking_num-1)+sample_num+1:end);

%%
figure;
if visualize_colmap
    [DenseColmapPointsXYZ, DenseColmapPointsRGB] = readXYZ_colorFile(datasetPath, 1);
    numPointsDenseColmap = length(DenseColmapPointsXYZ);

    tform = rigidtform3d;

    DenseColmapPointsXYZCloud = pointCloud(DenseColmapPointsXYZ.');
    DenseColmapPointsXYZCloud = pctransform(DenseColmapPointsXYZCloud, tform);

    scatter3(DenseColmapPointsXYZCloud.Location(:, 1), DenseColmapPointsXYZCloud.Location(:, 2), DenseColmapPointsXYZCloud.Location(:, 3), 10*ones(numPointsDenseColmap,1), (DenseColmapPointsRGB ./ 255).','.', 'UserData', 'dense colmap pointcloud');
    hold on;
end

for i = 1:tracking_num
    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 1, 'r'); hold on; %estimated with optimization
    plot_camera_frame(T_iss_cam{i}(1:3, 1:3), T_iss_cam{i}(1:3, 4), 1, 'k'); % true cam pose
    plot_camera_frame(T_cam_world{i}(1:3, 1:3), T_cam_world{i}(1:3, 4), 1, 'b'); hold on; %estimated with optimization
    plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 1, 'm');
end
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
% ylim([-15 -5]);
% zlim([4 10]);
% xlim([10.2 14]);
title("Tracking Optimization");
f=FigureRotator();
%%
figure;
plot3(PM_Map_ba(1, :), PM_Map_ba(2, :), PM_Map_ba(3, :), 'm', 'MarkerSize', 4); hold on
plot3(Win_Map_ba(1, :), Win_Map_ba(2, :), Win_Map_ba(3, :), 'm', 'MarkerSize', 4);

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 4);
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'b', 'MarkerSize', 4);

% for i = 1:tracking_num
%     plot_camera_frame(EstimatedPose_camframe{i}(1:3, 1:3), EstimatedPose_camframe{i}(1:3, 4), 1, 'r'); hold on; %estimated with optimization
%     plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 1, 'm');
%     plot_camera_frame(T_cam_p2c{i}(1:3, 1:3), T_cam_p2c{i}(1:3, 4), 1, 'b'); hold on; %estimated with optimization
% end

grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
% ylim([-15 -5]);
% zlim([4 10]);
% xlim([10.2 14]);
title("Tracking Optimization");
f=FigureRotator();

%%
function F = EReprojection_motiononly(params, PM_Point3D, Win_Point3D, Qpm, Qwin, K)
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

function F = EReprojection_localmapping(params, tracking_num, sample_num, Qpm, Qwin, K)
    Rk = cell(1, tracking_num);
    Tk = cell(1, tracking_num);
    for i = 1:tracking_num
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
