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

imagePath = 'data/td_yaw/gray/';
ellipsePath = 'data/td_yaw/results_p2c/';
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
posePath = 'data/td_yaw/groundtruth.txt';

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

true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:length(ellipse_result1)
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 100000
            P_iss_cam_true(i, :) = true_pose(j, :);
            R_iss_cam_true{i} = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
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

%%
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
    %perspective_two_circle_without choosing도 하기
end



%% ================= Visualize Results =================

%% draw results


s_iss_circle = cell(1, length(ellipse_result));
p_iss_circle = cell(1, length(ellipse_result));
s_cam_circle = cell(1, length(ellipse_result));
p_cam_circle = cell(1, length(ellipse_result));
for i = 1:length(ellipse_result)
    [p_iss_circle1, s_iss_circle1] = convert_cam_to_iss_frame(T_iss_cam_true{i}, circle_centers{i}(:, 1), surface_normals{i}(:, 1));
    [p_iss_circle2, s_iss_circle2] = convert_cam_to_iss_frame(T_iss_cam_true{i}, circle_centers{i}(:, 2), surface_normals{i}(:, 2));
    
    if dot(s_iss_circle1, pm_normal) > dot(s_iss_circle2, pm_normal)
        s_iss_circle{i} = s_iss_circle1;
        p_iss_circle{i} = p_iss_circle1(1:3);
        s_cam_circle{i} = surface_normals{i}(:, 1);
        p_cam_circle{i} = circle_centers{i}(:, 1);
        disp('1');
    else
        s_iss_circle{i} = s_iss_circle2;
        p_iss_circle{i} = p_iss_circle2(1:3);
        s_cam_circle{i} = surface_normals{i}(:, 2);
        p_cam_circle{i} = circle_centers{i}(:, 2);
        disp('2');
    end
end
save data/circles_frames3.mat

% Evaluate Result
for i = 1:length(ellipse_result)
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
for i = 1:length(ellipse_result)
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
for i = 1:length(ellipse_result)
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
for i = 1:length(ellipse_result)
    figure(10); cla;
    ellipses_params = ellipse_result(i, 2:end);
    image = [imagePath, images(i).name];
    figure_detected_ellipse(image, ellipses_params); 
    text(15, 15, sprintf('#%d ', i), 'FontSize', 15, 'color', 'r'); hold on; grid on; axis equal
    refresh; pause(0.5); 
end
