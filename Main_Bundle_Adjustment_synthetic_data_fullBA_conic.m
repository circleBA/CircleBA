% Clear workspace
clear; clc; close all;
% posePath = 'data/td_yaw/groundtruth_selected.txt';
% imagePath = 'data/td_yaw/gray_selected/';

% posePath = 'data/ff_return_journey_forward/groundtruth_selected.txt';
% imagePath = 'data/ff_return_journey_forward/gray_selected/';
posePath = 'data/multiview_mixed_data/groundtruth.txt';
imagePath = 'data/synthetic/';
% 원래 이걸로되어있음
% posePath = 'data/multiview_mixed_data/groundtruth.txt';
% imagePath = 'data/multiview_mixed_data/gray/';

% posePath = 'data/multiview_mixed_data_syn/groundtruth.txt';
% imagePath = 'data/multiview_mixed_data_syn/gray/';

%% ToDO
% 트래킹 넘을 2장 이상으로 해야하고 -  완
% 내 생각에 문제는 synthetic에서조차 initial map을 시각화했을때 두 원이 같은 평면상에 있지 않음..! - 해결함
% T_p2c_cam_true{2} 가 혼자 y값이 이상함 - 완

%% Step1: Generate Synthetic Circle Datadd

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

%% Step 2-3: Full Bundle Adjustment 
disp("optimization");
load('EstimatedPose.mat');

%1) xyzpoint 생성
num_frames = length(PM_center_cam); 
xyzPoints=[];
P_all = zeros(num_frames, 3);
for i = 1:num_frames
    P_cam = PM_center_cam{i};
    T_c2w = EstimatedPose{i};
    P_cam_h = [P_cam(:); 1];
    P_world = T_c2w * P_cam_h;
    P_all(i, :) = P_world(1:3)';
end
xyzPoints(1, :) = mean(P_all, 1);

W_all = zeros(num_frames, 3);
for i = 1:num_frames
    P_cam = Window_center_cam{i};
    T_c2w = EstimatedPose{i};
    P_cam_h = [P_cam(:); 1];
    P_world = T_c2w * P_cam_h;
    W_all(i, :) = P_world(1:3)';
end
xyzPoints(2, :) =  mean(W_all, 1);% 아주 근처 


% for i = 1:num_frames
%     % 카메라 좌표계 기준의 3D 포인트
%     P_cam = PM_center_cam{i};    % [1x3], i번째 프레임에서의 3D 위치
% 
%     % 해당 프레임의 카메라 pose
%     R = poses{i}.R;            % [3x3] 회전행렬
%     t = poses{i}.t;            % [3x1] 이동벡터
% 
%     % 월드 좌표계로 변환
%     P_world = (R * P_cam')' + t';   % [1x3]
% 
%     % 저장
%     P_world_all(i, :) = P_world;
% end
% 
% % 평균을 이용한 초기 xyzPoint 생성
% xyzPoints = mean(P_world_all, 1);  % [1 x 3]

% 
% % Initialize xyzPoints as an empty matrix
% xyzPoints = [];
% 
% % Loop through each frame and append the 3D points from PM_center_cam{i}
% for i = 1:num_frames
%     % Assuming PM_center_cam{i} is a 3x1 vector (for each frame)
%     xyzPoints = [xyzPoints; PM_center_cam{i}'];
% end

%2) points2D 생성
num_frames = length(ellipses_params);  
points2D = [];
points2Dw=[];

for i = 1:num_frames
    y = ellipses_params{i}(1, 1);
    x = ellipses_params{i}(1, 2);
    
    points2D = [points2D; x, y];
end

for i = 1:num_frames
    y = ellipses_params{i}(2, 1);
    x = ellipses_params{i}(2, 2);
    
    points2Dw = [points2Dw; x, y];
end

num_points = 2;  % 5개의 3D 점
viewIds = uint32(1:num_frames);
% pointTrack 배열 초기화
pointTracks = pointTrack.empty(num_points, 0);
% pointTracks = pointTrack(viewIds, points2D);
% % 각 포인트에 대해 pointTrack 객체 생성
pointTracks(1) = pointTrack(viewIds, points2D);
pointTracks(2) = pointTrack(viewIds, points2Dw);
% for i = 1:2
%     pointTracks(i) = pointTrack(viewIds, points2D);
% end

%3)posetable 생성
cameraPosesTable = table('Size', [num_frames, 2], 'VariableTypes', {'uint32', 'rigid3d'}, ...
                        'VariableNames', {'ViewId', 'AbsolutePose'});

for i = 1:num_frames
    T = EstimatedPose{i};
    R = T(1:3, 1:3);  
    t = T(1:3, 4);  
    % C = -R' * t;
    % T = [R, t; 0, 0, 0, 1]; 
    cameraPoseObj = rigid3d(R, t');
    % R_w2c = R';
    % t_w2c = -R' * t;
    % cameraPoseObj = rigid3d(R_w2c, t_w2c');


    cameraPosesTable.ViewId(i) = uint32(i);
    cameraPosesTable.AbsolutePose(i) = cameraPoseObj;
end

%debug

for i = 1:length(pointTracks)
    fprintf("pointTrack %d:\n", i);
    disp([pointTracks(i).ViewIds', pointTracks(i).Points]);
end
K=intrinsics.K;

for i = 1:length(pointTracks)
    pts2D = pointTracks(i).Points;
    viewIds = pointTracks(i).ViewIds;

    for j = 1:length(viewIds)
        pose = cameraPosesTable.AbsolutePose(cameraPosesTable.ViewId == viewIds(j));
        R = pose.Rotation;
        t = pose.Translation';

        pt3D = xyzPoints(i, :)';  % [3x1]
        pt_proj = K * (R * pt3D + t);
        pt_proj = pt_proj(1:2) / pt_proj(3);

        err = norm(pt_proj - pts2D(j,:)');
        fprintf("point %d frame %d → error: %.2f px\n", i, viewIds(j), err);
    end
end

[xyzRefinedPoints, refinedPoses] = bundleAdjustment(xyzPoints, pointTracks, cameraPosesTable, intrinsics,'FixedViewIDs', 1);



% figure;
% pcshowpair(pointCloud(xyzRefinedPoints), pointCloud(xyzRefinedPoints), ...
%     'AxesVisibility', 'on', 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 40);
% hold on;
% 
% plotCamera(refinedPoses, 'Size', 0.1, 'Color', 'g');  

save('refinedPoses.mat', 'refinedPoses');

% title('Refined 3D Points and Camera Poses');
% hold off;
EstimatedPose1 = cell(num_frames, 1);
for i = 1:num_frames
    % ViewId 기준으로 pose 추출
    pose = refinedPoses.AbsolutePose(i);

    % R, t 가져오기
    R = pose.Rotation;
    t = pose.Translation';  % col vector로 변환

    % 4x4 pose 행렬로 구성
    T = [R, t; 0 0 0 1];

    EstimatedPose1{i} = T;
end

% 저장
save('EstimatedPose_after_BA.mat', 'EstimatedPose1');



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

function residuals = fullBA_residual(params_all, PM_set, Win_set, C_pm_set, C_win_set, ellipses_set, PM_center_set, K,T_p2c_cam)
    num_frames = length(C_pm_set);
    sample_num = size(PM_set, 2);

    lamda_conic = 1 / sample_num;
    lamda_center = 1;
    residuals=[];

    residuals1 = zeros(sample_num,1);
    residuals2= zeros(num_frames,1);
    poses = cell(1, num_frames);
    poses{1} = T_p2c_cam; % First frame is fixed to identity

    % Initialization
    for i = 2:num_frames
        xi = params_all((i-2)*6+1:(i-1)*6);
        poses{i} = expSE3(xi);
    end

    for i = 2:num_frames
        T = poses{i};
        center_pt = [PM_center_set{1}; 1];
        center_proj = T * center_pt;
        center_proj_2d = K * center_proj(1:3);
        center_proj_2d = center_proj_2d / center_proj_2d(3);

        ref_x = ellipses_set{i}(1,2);
        ref_y = ellipses_set{i}(1,1);
        reference = [ref_x; ref_y];

        center_error = norm(center_proj_2d(1:2) - reference);
        residuals2(i) = lamda_center * center_error;

        % % Conic residuals
        % for j = 1:sample_num
        %     pt_pm = [PM_set(:, j); 1];
        %     pt_win = [Win_set(:, j); 1];
        % 
        %     pt_pm_trans = T * pt_pm;
        %     pt_win_trans = T * pt_win;
        % 
        %     pt_pm_proj = K * pt_pm_trans(1:3);
        %     pt_win_proj = K * pt_win_trans(1:3);
        % 
        %     pt_pm_proj = pt_pm_proj / pt_pm_proj(3);
        %     pt_win_proj = pt_win_proj / pt_win_proj(3);
        % 
        %     val_pm = pt_pm_proj' * C_pm_set{i} * pt_pm_proj;
        %     val_win = pt_win_proj' * C_win_set{i} * pt_win_proj;
        % 
        %     residuals1(j) = (val_pm + val_win);
        % end

        % Center residual
        
     end
    residuals=[residuals1,residuals2];
    residuals = [residuals; residuals1];
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