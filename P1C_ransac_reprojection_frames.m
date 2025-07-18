clc;
close all;
clear variables; %clear classes;

%%
load('data/circles_frames3.mat');
sample_num = 1000;
% two solution is circle_centers, surface_normals
% chosen right solution is p_cam_circle, s_cam_circle
%% Cone Equation
% Visualize the Initial Map of cone
figure;
plot_inertial_frame(1); hold on; % Camera Frame
k = ellipse_result(1, 2); %y
h = ellipse_result(1, 3); %x
a = ellipse_result(1, 4)/2; %a
b = ellipse_result(1, 5)/2; %b
theta = 90-ellipse_result(1, 6); %angle (deg)
t = linspace(0, 2*pi, sample_num);
x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y_ellipse = k + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
x_cam = (x_ellipse - intrinsics.K(1, 3)) / intrinsics.K(1, 1);
y_cam = (y_ellipse - intrinsics.K(2, 3)) / intrinsics.K(2, 2);
% Define the rays (from camera center through each ellipse point)
num_points = length(x_cam);
rays = zeros(3, num_points);
scaling_factor = 10;
for i = 1:num_points
    % Each ray in normalized camera coordinates
    rays(:, i) = [x_cam(i); y_cam(i); 1];
    % Normalize the ray
    rays(:, i) = rays(:, i) / norm(rays(:, i));
    rays(:, i) = scaling_factor * rays(:, i);
end
% Plot the rays
for i = 1:num_points
    plot3([0 rays(1, i)], ...
          [0 rays(2, i)], ...
          [0 rays(3, i)], 'g-');
end
% Plot the ellipse points on the image plane (z = 1)
plot3(x_cam, y_cam, ones(1, num_points), 'ro');

% 3D circle
if abs(s_cam_circle{1}(1, 1)) < abs(s_cam_circle{1}(2, 1)) && abs(s_cam_circle{1}(1, 1)) < abs(s_cam_circle{1}(3, 1))
    v = [1, 0, 0];
elseif abs(s_cam_circle{1}(2, 1)) < abs(s_cam_circle{1}(1, 1)) && abs(s_cam_circle{1}(2, 1)) < abs(s_cam_circle{1}(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(s_cam_circle{1}(:, 1), v);
u = u / norm(u);
v = cross(s_cam_circle{1}(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
circle1 = R * (u' * cos(theta) + v' * sin(theta));
plot3(p_cam_circle{1}(1, 1) + circle1(1, :), p_cam_circle{1}(2, 1) + circle1(2, :), p_cam_circle{1}(3, 1) + circle1(3, :), 'b', 'LineWidth', 4);hold on;

axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
ylim([-10 10]);
zlim([-10 10]);
xlim([-10 10]);
title('3D Visualization of Cone in Camera Frame');
f = FigureRotator();



%% make point-ray correspondence
%Initial Circle map 
InitialMap1 = [p_cam_circle{1}(1, 1) + circle1(1, :); p_cam_circle{1}(2, 1) + circle1(2, :); p_cam_circle{1}(3, 1) + circle1(3, :)];
% InitialMap2 = [circle_centers(1, 2) + circle2(1, :); circle_centers(2, 2) + circle2(2, :); circle_centers(3, 2) + circle2(3, :)]; 


%% Reproject test with true pose
num = length(ellipse_result);
Relative_Trans_opt = cell(1, num);
R_true = cell(1, num);
T_true = cell(1, num);
R_relative_opt = cell(1, num);
T_relative_opt = cell(1, num);

T_last_inv = inv(T_iss_cam{1});
T_relative = T_last_inv * T_iss_cam{2};
T_relative = inv(T_relative);
R_true1 = T_relative(1:3, 1:3);
T_true1 = T_relative(1:3, 4);
R_init = R_true1;
T_init = T_true1;
% R_init = eye(3);
% T_init = [0;0;0];

for i = 2:length(ellipse_result)
    % value setting
    Qe = C_quad{i};
    T_last_inv = inv(T_iss_cam{i-1});
    T_relative = T_last_inv * T_iss_cam{i};
    T_relative = inv(T_relative);
    R_true{i} = T_relative(1:3, 1:3);
    T_true{i} = T_relative(1:3, 4);

    %optimization per frame
    objective_function = @(params) EReprojection(params, InitialMap1, Qe, intrinsics.K);
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
    initial_params = [R_init, T_init];
    [opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);
    
    R_relative_opt{i} = opt_params(1:3, 1:3);
    T_relative_opt{i} = opt_params(1:3, 4);
    Relative_Trans_opt{i} = [R_relative_opt{i}, T_relative_opt{i}; 0 0 0 1];
end

%% Visualize result
%True
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
xlim([10 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame (true camera pose)');

f = FigureRotator();


%Estimated after LM optimization
% Initialize the absolute rotation and translation
R_esti = cell(1, num);
T_esti = cell(1, num);
T_iss_cam_esti = cell(1, num);
T_iss_cam_esti{1} = T_iss_cam{1};
% Calculate the absolute positions
for i = 2:num
    T_iss_cam_esti{i} = T_iss_cam_esti{i-1}*Relative_Trans_opt{i};
end
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
    plot_camera_frame(T_iss_cam_esti{i}(1:3, 1:3), T_iss_cam_esti{i}(1:3, 4), 1, 'k'); hold on;
    plot3(p_iss_circle{i}(1), p_iss_circle{i}(2), p_iss_circle{i}(3), 'bo'); 
    quiver3(p_iss_circle{i}(1), p_iss_circle{i}(2), p_iss_circle{i}(3), s_iss_circle{i}(1), s_iss_circle{i}(2), s_iss_circle{i}(3), 'b');
end
% plot_camera_frame(T_iss_cam_esti{2}(1:3, 1:3), T_iss_cam_esti{2}(1:3, 4), 1, 'k'); hold on;
% plot_camera_frame(T_iss_cam_esti{1}(1:3, 1:3), T_iss_cam_esti{1}(1:3, 4), 1, 'r'); hold on;

xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([8 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame (estimated camera pose)');
f = FigureRotator();
%%
function F = EReprojection(params, Point3D, Qe, K)
    Rk = params(1:3, 1:3);
    Tk = params(1:3, 4);
    n = size(Point3D, 2);
    F = zeros(n, 1);
    for i = 1:n
        % 3D point
        point3D = Point3D(:, i);
        projected_point = K * (Rk * point3D + Tk);
        projected_point = projected_point(1:2) / projected_point(3);
        error = [projected_point; 1]' * Qe * [projected_point; 1];
        F(i) = error;
    end
    F = sum(F);
end



function [c,ceq] = nlcon(x)
ceq = [];
c = sin(x(1)) - cos(x(2));
end