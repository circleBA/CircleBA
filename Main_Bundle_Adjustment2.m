clc;
close all;
clear variables; %clear classes;

%% preprocessing - only detected ellipse

% imagePath = 'data/td_yaw/gray/';
% ellipsePath = 'data/td_yaw/results_p2c/';
% datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
% posePath = 'data/td_yaw/groundtruth.txt';

imagePath = 'data/ff_return_journey_forward/gray/';
ellipsePath = 'data/ff_return_journey_forward/results_p2c1/';
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_yaw/';
posePath = 'data/ff_return_journey_forward/groundtruth.txt';

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

        if two_ellipse_result(j, 7) == 0 && two_ellipse_result(j, 2) ~= 0
            ellipse1 = two_ellipse_result(j, 1:6);
        end

        if two_ellipse_result(j, 7) == 1 && two_ellipse_result(j, 2) ~= 0
            ellipse2 = two_ellipse_result(j, 1:6);
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
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 10
            P_iss_cam_true(i, :) = true_pose(j, :);
            R_iss_cam_true{i} = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
        end
    end
end

tracking_num = size(ellipse_result1, 1);
sample_num = 100;

%% Input Params Setting
% R = [0.55, 0.145]; % real hatch radius
R = [0.5455 0.145];
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

    if angle_diff{i} > 2
        ransac(i) = 0;
    % else
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
    Top = [0 1 0 0; 1 0 0 0; 0 0 -1 0; 0 0 0 1];
    T_o_cam{i} = Top * inv(T_cam_p2c{i});
    T_o_p2c{i} = T_o_cam{i} * T_cam_p2c{i}; 
    

    PM_normal_o{i} = T_o_cam{i}(1:3, 1:3) * PM_normal_cam{i}; %+ T_o_cam{i}(1:3, 4);
    Window_normal_o{i} = T_o_cam{i}(1:3, 1:3) * Window_normal_cam{i}; % + T_o_cam{i}(1:3, 4);
    PM_center_o{i} = T_o_cam{i}(1:3, 1:3) * PM_center_cam{i} + T_o_cam{i}(1:3, 4);
    Window_center_o{i} = T_o_cam{i}(1:3, 1:3) * Window_center_cam{i} + T_o_cam{i}(1:3, 4);
    % PM_normal_o{i} = PM_normal_o{i}/norm(PM_normal_o{i});
    % Window_normal_o{i} = Window_normal_o{i}/norm(Window_normal_o{i});


    % T_origin_cam{i} = T_

    % T_p2c1_p2ci{i} = inv(T_origin_p2c{1})*T_origin_p2c{i};
    % 
    % T_origin_p2c_fixed{i} = T_origin_p2c{i} * inv(T_p2c1_p2ci{i});
    % % T_p2c_cam_fixed{i} = T_p2c1_p2ci{i} * T_p2c_cam{i}; % 여기가 이상한건지, 맞게 되었는데 p2c가 이상해서 시각화가 이상한건지..?
    % T_cam_p2c{i} = T_cam_p2c{i} * inv(T_p2c1_p2ci{i});
    % T_p2c_cam_fixed{i} = Top * inv(T_cam_p2c{i});

    z = ( PM_center_o{i} - Window_center_o{i} ) / norm(PM_center_o{i} - Window_center_o{i});
    x = cross(PM_normal_o{i}, z);
    x = x/norm(x);
    z = cross(x, PM_normal_o{i});
    z = z/norm(z);
    y = PM_normal_o{i};
    R_o_p2c_new = [x, y, z];
    t_o_p2c_new = PM_center_o{i};
    T_o_p2c_new{i} = [R_o_p2c_new, t_o_p2c_new; 0 0 0 1];
    T_p2c1_p2ci{i} = inv(T_o_p2c_new{1})*T_o_p2c_new{i};
    T_o_p2c_fixed{i} = T_o_p2c_new{i} * inv(T_p2c1_p2ci{i});
    T_o_cam_fixed{i} = T_o_cam{i} * T_p2c1_p2ci{i};
    % T_o_cam_fixed{i} = T_o_p2c_fixed{i} * T_o_cam{i};
    % T_o_cam_fixed{i} = T_o_p2c_fixed{i} * inv(T_cam_p2c{i});
    % T_o_cam_fixed{i} = T_o_cam{i} * inv(T_p2c1_p2ci{i});
    % T_o_cam_fixed{i} = inv(T_p2c1_p2ci{i}) * inv(T_o_p2c_new{1}) * inv(T_cam_p2c{i});

    % if Window_center_o{i}(1) < 0
    %     dd
    % 
    % end



    % PM_normal_fixed{i} = T_o_cam_fixed{i}(1:3, 1:3) * PM_normal_o{i} + T_o_cam{i}(1:3, 4);
    % Window_normal_o{i} = T_o_cam{i}(1:3, 1:3) * Window_normal_cam{i} + T_o_cam{i}(1:3, 4);
    % PM_center_o{i} = T_o_cam{i}(1:3, 1:3) * PM_center_cam{i} + T_o_cam{i}(1:3, 4);
    % Window_center_o{i} = T_o_cam{i}(1:3, 1:3) * Window_center_cam{i} + T_o_cam{i}(1:3, 4);
    % PM_normal_o{i} = PM_normal_o{i}/norm(PM_normal_o{i});
    % Window_normal_o{i} = Window_normal_o{i}/norm(Window_normal_o{i});
    
    % T_o_cam_fixed{i} = T_o_p2c_fixed{i} * T_o_cam{i};
end


% k=10;
ransac = logical(ransac);
tstamp = tstamp(ransac);
imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
T_p2c_cam = T_p2c_cam(ransac);
T_o_cam = T_o_cam(ransac);
T_cam_p2c = T_cam_p2c(ransac);
T_o_cam_fixed = T_o_cam_fixed(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);
PM_normal_o = PM_normal_o(ransac);
PM_center_o = PM_center_o(ransac);
Window_normal_o = Window_normal_o(ransac);
Window_center_o = Window_center_o(ransac);
T_o_p2c = T_o_p2c(ransac);
T_o_p2c_new = T_o_p2c_new(ransac);
T_o_p2c_fixed = T_o_p2c_fixed(ransac);

tracking_num = size(R_iss_cam_true, 2);

T_p2c_iss = T_o_cam{1} * inv(T_iss_cam_true{1});
for i = 1:tracking_num
    T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
end
k=10;
figure;
for i = 1:tracking_num
    plot_camera_frame(T_o_cam{i}(1:3, 1:3), T_o_cam{i}(1:3, 4), 0.5, 'b'); hold on;
    plot_camera_frame(T_o_cam_fixed{i}(1:3, 1:3), T_o_cam_fixed{i}(1:3, 4), 0.5, 'r'); hold on;
    plot_camera_frame(T_o_p2c{i}(1:3, 1:3), T_o_p2c{i}(1:3, 4), 0.5, 'k'); hold on;
end
% plot_camera_frame(T_p2c_cam{1}(1:3, 1:3), T_p2c_cam{1}(1:3, 4), 0.5, 'r'); hold on;
axis equal; grid on;
f = FigureRotator();

k=10;
figure;
for i = 1:tracking_num
    u = T_o_p2c{i}(1:3, 1)';
    v = T_o_p2c{i}(1:3, 3)';
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));

    u = T_o_p2c_new{i}(1:3, 1)';
    v = T_o_p2c_new{i}(1:3, 3)';
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D_new = R(1) * (u' * cos(theta) + v' * sin(theta));

    u = T_o_p2c_fixed{i}(1:3, 1)';
    v = T_o_p2c_fixed{i}(1:3, 3)';
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D_fixed = R(1) * (u' * cos(theta) + v' * sin(theta));

  

    plot3(T_o_p2c{i}(1, 4) + PM_circle_3D(1, :), T_o_p2c{i}(2, 4) + PM_circle_3D(2, :), T_o_p2c{i}(3, 4) + PM_circle_3D(3, :), 'b', 'LineWidth', 2);hold on;
    % plot3(T_o_p2c_new{i}(1, 4) + PM_circle_3D_new(1, :), T_o_p2c_new{i}(2, 4) + PM_circle_3D_new(2, :), T_o_p2c_new{i}(3, 4) + PM_circle_3D_new(3, :), 'r', 'LineWidth', 2);hold on;
    plot3(T_o_p2c_fixed{i}(1, 4) + PM_circle_3D_fixed(1, :), T_o_p2c_fixed{i}(2, 4) + PM_circle_3D_fixed(2, :), T_o_p2c_fixed{i}(3, 4) + PM_circle_3D_fixed(3, :), 'r', 'LineWidth', 2);hold on;
    
    % plot_camera_frame(T_o_cam{i}(1:3, 1:3), T_o_cam{i}(1:3, 4), 0.5, 'b'); hold on;
    plot_camera_frame(T_o_cam_fixed{i}(1:3, 1:3), T_o_cam_fixed{i}(1:3, 4), 0.5, 'm'); hold on;

    plot3(PM_center_o{i}(1), PM_center_o{i}(2), PM_center_o{i}(3), 'co'); 
    quiver3(PM_center_o{i}(1), PM_center_o{i}(2), PM_center_o{i}(3), PM_normal_o{i}(1), PM_normal_o{i}(2), PM_normal_o{i}(3), 'c');
    plot3(Window_center_o{i}(1), Window_center_o{i}(2), Window_center_o{i}(3), 'mo'); 
    quiver3(Window_center_o{i}(1), Window_center_o{i}(2), Window_center_o{i}(3), Window_normal_o{i}(1), Window_normal_o{i}(2), Window_normal_o{i}(3), 'm');

    if Window_center_o{i}(1)<0
        % plot_camera_frame(T_o_cam{i}(1:3, 1:3), T_o_cam{i}(1:3, 4), 0.5, 'r'); hold on;
        plot_camera_frame(T_o_cam_fixed{i}(1:3, 1:3), T_o_cam_fixed{i}(1:3, 4), 0.5, 'r'); hold on;
    
        plot3(PM_center_o{i}(1), PM_center_o{i}(2), PM_center_o{i}(3), 'co'); 
        quiver3(PM_center_o{i}(1), PM_center_o{i}(2), PM_center_o{i}(3), PM_normal_o{i}(1), PM_normal_o{i}(2), PM_normal_o{i}(3), 'b');
        plot3(Window_center_o{i}(1), Window_center_o{i}(2), Window_center_o{i}(3), 'mo'); 
        quiver3(Window_center_o{i}(1), Window_center_o{i}(2), Window_center_o{i}(3), Window_normal_o{i}(1), Window_normal_o{i}(2), Window_normal_o{i}(3), 'b');
    end
end
plot_camera_frame(T_o_cam{1}(1:3, 1:3), T_o_cam{1}(1:3, 4), 0.5, 'm'); hold on;
xlabel('x'); ylabel('y'); zlabel('z');
axis equal; grid on;
f = FigureRotator();



k=10;



%% ALL P2C Map Test
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
    raysi = zeros(4, num_points);
    ellipse = zeros(3, num_points);
    ellipsei = zeros(4, num_points);
    scaling_factor = 10;
    for j = 1:num_points
        % Each ray in normalized camera coordinates
        rays(:, j) = [x_cam(j); y_cam(j); 1];
        ellipse(:, j) = [x_cam(j); y_cam(j); 1];
        % Normalize the ray
        rays(:, j) = rays(:, j) / norm(rays(:, j));
        rays(:, j) = scaling_factor * rays(:, j);
        raysi(:, j) = inv(T_cam_p2c{1}) * [rays(:, j); 1];
        ellipsei(:, j) = inv(T_cam_p2c{1}) * [ellipse(:, j); 1];
    end
    % Plot the rays
    % for j = 1:2:num_points
    %     plot3([T_p2c_cam{1}(1, 4) raysi(1, j)], ...
    %           [T_p2c_cam{1}(2, 4) raysi(2, j)], ...
    %           [T_p2c_cam{1}(3, 4) raysi(3, j)], 'g-');
    % end
    % Plot the ellipse points on the image plane (z = 1)
    % plot3(ellipsei(1, :), ellipsei(2, :), ellipsei(3, :), 'mo');
end
%PM circle


for i = 1:tracking_num
    %Tcam1_p2ci
    % Tcam1cam2 = inv(T_p2c_cam{1}) * T_p2c_cam{i};
    Tcam1cam2 = inv(T_p2c1_p2ci{i});
    moved = Tcam1cam2 * [PM_center_cam{i}; 1];
    PM_center_cam{i} = moved(1:3);
    moved = Tcam1cam2 * [PM_normal_cam{i}; 1];
    PM_normal_cam{i} = moved(1:3)/norm(moved(1:3));
    moved = Tcam1cam2 * [Window_center_cam{i}; 1];
    Window_center_cam{i} = moved(1:3);
    moved = Tcam1cam2 * [Window_normal_cam{i}; 1];
    Window_normal_cam{i} = moved(1:3)/norm(moved(1:3));


    angle_diff2(i) = acos(dot(PM_normal_cam{1}, PM_normal_cam{i})/norm(PM_normal_cam{i}))*180/pi;

    if abs(PM_normal_cam{i}(1, 1)) < abs(PM_normal_cam{i}(2, 1)) && abs(PM_normal_cam{i}(1, 1)) < abs(PM_normal_cam{i}(3, 1))
        v = [1, 0, 0];
    elseif abs(PM_normal_cam{i}(2, 1)) < abs(PM_normal_cam{i}(1, 1)) && abs(PM_normal_cam{i}(2, 1)) < abs(PM_normal_cam{i}(3, 1))
        v = [0, 1, 0];
    else
        v = [0, 0, 1];
    end
    u = cross(PM_normal_cam{i}(:, 1), v);
    u = u / norm(u);
    v = cross(PM_normal_cam{i}(:, 1), u);
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
    plot3(PM_center_cam{i}(1, 1) + PM_circle_3D(1, :), PM_center_cam{i}(2, 1) + PM_circle_3D(2, :), PM_center_cam{i}(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 2);hold on;
    Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
    plot3(Window_center_cam{i}(1, 1) + Window_circle_3D(1, :), Window_center_cam{i}(2, 1) + Window_circle_3D(2, :), Window_center_cam{i}(3, 1) + Window_circle_3D(3, :), 'm', 'LineWidth', 2);hold on;



    plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'r'); hold on;
    pause(1);
    view(45, 45);
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end
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
k=10;

%%
u = [0, 1, 0];
v = [0, 0, 1];
theta = linspace(0, 2*pi, sample_num);
PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
InitialMap_PM = [PM_circle_3D(1, :); PM_circle_3D(2, :); PM_circle_3D(3, :)];
InitialMap_Win = [Window_circle_3D(1, :); Window_circle_3D(2, :); (0.3470) + Window_circle_3D(3, :)];

figure;
for i = 1:tracking_num
    plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b'); hold on;
    plot_camera_frame(T_p2c_cam_fixed{i}(1:3, 1:3), T_p2c_cam_fixed{i}(1:3, 4), 0.5, 'r'); hold on;
    plot_camera_frame(T_origin_p2c_fixed{i}(1:3, 1:3), T_origin_p2c_fixed{i}(1:3, 4), 0.5, 'k'); hold on;
end
plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 4);
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'b', 'MarkerSize', 4);
xlabel('x'); ylabel('y'); zlabel('z');
axis equal; grid on;
f = FigureRotator();

%% Tracking Optimization
disp("optimization");
ransac2 = ones(1, tracking_num);
EstimatedPose{1} = T_p2c_cam{1};
PM_normal_i = EstimatedPose{1} * [PM_normal_cam{1}; 1];
PM_normal_init = PM_normal_i(1:3)/norm(PM_normal_i(1:3));
figure;
for i = 2:tracking_num
    T_relative = inv( inv(T_p2c_cam{1}) * T_p2c_cam{i} );
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
    
    EstimatedPose{i} = T_p2c_cam{1} * inv( Relative_Trans_opt );
    
    
    PM_normal = EstimatedPose{i} * [PM_normal_cam{i}; 1];
    PM_normal_esti = PM_normal(1:3) / norm(PM_normal(1:3));
    angle_diff2{i} = acos(dot(PM_normal_init, PM_normal_esti))*180/pi;
    % if angle_diff2{i} > 30
    %     ransac2(i) = 0;
    % end
    
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


    tstampStr = sprintf('%.0f', tstamp(i));
    image = strcat(imagePath, imgList(i, :));
    imshow(image); hold on;
    plot(u1, v1, 'co', 'MarkerSize',1); hold on;% Plot reprojected points in red
    plot(u2, v2, 'mo', 'MarkerSize',1); % Plot reprojected points in red
    pause(1);
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

figure;
plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'b', 'MarkerSize', 4); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'b', 'MarkerSize', 4);

for i = 1:tracking_num
    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
    plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    % plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 0.5, 'm');
    grid on; axis equal;
    % pause(1);
end
xlabel('x'); ylabel('y'); zlabel('z');
axis equal; grid on;
f = FigureRotator();
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


function F = EReprojectionBackprojection_localmapping(params, tracking_num, sample_num, Qpm, Qwin, PM_normal_cam, PM_center_cam, Window_normal_cam, Window_center_cam, K)
    Rk = cell(1, tracking_num);
    Tk = cell(1, tracking_num);
    Ti = cell(1, tracking_num);
    for i = 1:tracking_num
        Ti{i} = [params(1:3, i*4-3:i*4); 0 0 0 1];
        Rk{i} = Ti{i}(1:3, 1:3);
        Tk{i} = Ti{i}(1:3, 4);
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
