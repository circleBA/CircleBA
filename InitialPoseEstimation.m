clc;
close all;
clear variables; %clear classes;

%% Initial pose Estimation using two frame

% ------------------------------------------------------------------------------
% Function : perspective-1-circle in ISS
% Project  : Astrobee ISS Datasets
% Author   : suyoungkang
% Version  : V01 
% Comment  : https://astrobee-iss-dataset.github.io/
% Status   : working
% ------------------------------------------------------------------------------

clc;
close all;
clear variables; %clear classes;
delimiter = ' ';

%% Input Params Setting

imagePath = 'data/td_yaw_two_view/gray_aamed/';
ellipsePath = 'data/td_yaw_two_view/results.txt';
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
posePath = 'data/td_yaw_two_view/groundtruth.txt';

ellipse_result = readmatrix(ellipsePath, 'Delimiter', ' ');
true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(ellipse_result, 1)
    for j = 1:length(true_pose)
        if abs(ellipse_result(i, 1) - true_pose(j, 1)) < 10
            P_iss_cam(i, :) = true_pose(j, :);
        end
    end
end

R = 0.6; % real hatch radius
% true circle position
pm_position = [10.9349; -10.5572; 5.2508];
pm_normal = [0; 1; 0];

visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

% rgb.txt읽어서 그거대로 이미지랑 gt pose 저장해서 거기서 루프 돌면서 circle surface 저장하게 하고
% 마지막에 각자의 circle을 inertia frame으로 고정해서 cam pose그리게 만들기

%% P1C
circle_centers = cell(1, size(ellipse_result, 1));
surface_normals = cell(1, size(ellipse_result, 1));
T_iss_cam = cell(1, size(ellipse_result, 1));
for i = 1:size(ellipse_result, 1)
    % Set up params
    tstamp = ellipse_result(i, 1);
    ellipses_params = ellipse_result(i, 2:end);
    % image = imagePath + str(tstamp) + '.png';
    R_iss_cam{i} = quat2rotm([P_iss_cam(i, 8), P_iss_cam(i, 5:7)]);
    t_iss_cam{i} = P_iss_cam(i, 2:4)';
    T_iss_cam{i} = [R_iss_cam{i}, t_iss_cam{i}; 0 0 0 1];
    
    % Ellipse Equation
    k = ellipses_params(1); %y
    h = ellipses_params(2); %x
    a = ellipses_params(3)/2; %a
    b = ellipses_params(4)/2; %b
    angle = 90-ellipses_params(5); %angle (deg)
    % ellipse equation to quadratic form 
    [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
    C = [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F]; 

    % Cone Equation
    Q = intrinsics.K'* C * intrinsics.K;

    % eigen value and normalized eigen vector of Q
    [V, D] = eig(Q); % eigen value D, eigen vector V
    % Normalize the eigenvectors
    for j = 1:size(V, 2)
        V(:, j) = V(:, j) / norm(V(:, j));
    end
    
    [U, lam] = find_transform_cone_cam(V, D);

    % circle center has two solution
    circle_centers{i} = find_circle_center_position(U, lam, R);
    % surface normal has two solution corresponding 
    surface_normals{i} = find_circle_surface_normal(U, lam);
end

save data/circles_frames.mat



%%
rads = cell(1, size(ellipse_result, 1));
thetas = cell(1, size(ellipse_result, 1));
dists = cell(1, size(ellipse_result, 1));
for i = 1:size(ellipse_result, 1)
    for j = 1:2
        rads{i}(:, j) = arcos(dot(surface_normals{i}(:, j),[0;0;-1]));
        thetas{i}(:, j) = rad2deg(rads{i}(:, j));
        dists{i}(1, j) = sqrt(sum((circle_{i}-pm_position).^2));
    end
end

%% fixed circle
% fix circle to 0,0,0 and draw 
    % 일단 단순하게 2개 솔루션의 circle center가 같다고 가정 (실제로 오차가 매우 적으니까)
theta = linspace(0, 2*pi, 100);
xunit = R * cos(theta) + 0;
yunit = R * sin(theta) + 0;
zunit = zeros(1, length(theta));
figure;
plot_inertial_frame(1); hold on; % this is circle
plot3(xunit, yunit, zunit, 'r', 'LineWidth', 2); hold off;
axis equal; grid on; 
xlabel('X'); ylabel('Y'); zlabel('Z');
f = FigureRotator();

%% 

R = 0.6;








%% ================= Visualize Results =================

%% draw results

% translation_matrix = eye(4);
% translation_matrix(1:3, 4) = -circle_centers{i}(:, 1); %1 or 2
% 
s_iss_circle = cell(1, size(ellipse_result, 1));
p_iss_circle = cell(1, size(ellipse_result, 1));
s_cam_circle = cell(1, size(ellipse_result, 1));
p_cam_circle = cell(1, size(ellipse_result, 1));
for i = 1:size(ellipse_result, 1)
    [p_iss_circle1, s_iss_circle1] = convert_cam_to_iss_frame(T_iss_cam{i}, circle_centers{i}(:, 1), surface_normals{i}(:, 1));
    [p_iss_circle2, s_iss_circle2] = convert_cam_to_iss_frame(T_iss_cam{i}, circle_centers{i}(:, 2), surface_normals{i}(:, 2));
    
    if dot(s_iss_circle1, pm_normal) > dot(s_iss_circle2, pm_normal)
        s_iss_circle{i} = s_iss_circle1;
        p_iss_circle{i} = p_iss_circle1(1:3);
        s_cam_circle{i} = surface_normals{i}(:, 1);
        p_cam_circle{i} = circle_centers{i}(:, 1);
    else
        s_iss_circle{i} = s_iss_circle2;
        p_iss_circle{i} = p_iss_circle2(1:3);
        s_cam_circle{i} = surface_normals{i}(:, 2);
        p_cam_circle{i} = circle_centers{i}(:, 2);
    end
end

% Evaluate Result
for i = 1:size(ellipse_result, 1)
    center_error(i) = sqrt(sum((p_iss_circle{i}-pm_position).^2));
    surface_error(i) = acos(dot(s_iss_circle{i}/norm(s_iss_circle{i}),pm_normal/norm(pm_normal))) * 180 / pi;
end



% Visualization with true camera pose / estimated circle position
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
for i = 1:size(ellipse_result, 1)
    plot_camera_frame(R_iss_cam{i}, t_iss_cam{i}, 1, 'k'); hold on;
    plot3(p_iss_circle{i}(1), p_iss_circle{i}(2), p_iss_circle{i}(3), 'bo'); 
    quiver3(p_iss_circle{i}(1), p_iss_circle{i}(2), p_iss_circle{i}(3), s_iss_circle{i}(1), s_iss_circle{i}(2), s_iss_circle{i}(3), 'b');
end

xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([10.5 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame (true camera pose)');

f = FigureRotator();

% Visualization with true object pose / estimated camera position
for i = 1:size(ellipse_result, 1)
    % Calculate the translation to move the object to the origin
    translation = -p_cam_circle{i};
    
    % Calculate the rotation axis and angle
    rotation_axis = cross(s_cam_circle{i}, pm_normal);
    rotation_axis = rotation_axis / norm(rotation_axis);  % Normalize the rotation axis
    rotation_angle = acos(dot(s_cam_circle{i}, pm_normal) / (norm(s_cam_circle{i}) * norm(pm_normal)));
    
    % Create the rotation matrix using the axis-angle representation
    K = [0, -rotation_axis(3), rotation_axis(2);
         rotation_axis(3), 0, -rotation_axis(1);
         -rotation_axis(2), rotation_axis(1), 0];
    R = eye(3) + sin(rotation_angle) * K + (1 - cos(rotation_angle)) * (K * K);
    
    % Construct the homogeneous transformation matrix for the object
    T_cam_obj = [R, translation; 0, 0, 0, 1];
    
    % Calculate the camera position with respect to the object
    T_obj_cam{i} = inv(T_cam_obj);
    
end

%% show detected ellipses
image_files = fullfile(imagePath, '*.png');
images = dir(image_files);
figure(10);
pause(5);
for i = 1:size(ellipse_result, 1)
    figure(10); cla;
    ellipses_params = ellipse_result(i, 2:end);
    image = [imagePath, images(i).name];
    figure_detected_ellipse(image, ellipses_params); 
    refresh; pause(0.5); 
end
