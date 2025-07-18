clc;
close all;
clear variables; %clear classes;

%%
load('data/circle56.mat');
sample_num = 1000;
%% Cone Equation
% Visualize the cone
figure;
plot_inertial_frame(1); hold on; % Camera Frame
k = ellipses_params(1); %y
h = ellipses_params(2); %x
a = ellipses_params(3)/2; %a
b = ellipses_params(4)/2; %b
theta = 90-ellipses_params(5); %angle (deg)
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
% plot3(x_cam, y_cam, ones(1, num_points), 'ro');

%first circle
if abs(surface_normals(1, 1)) < abs(surface_normals(2, 1)) && abs(surface_normals(1, 1)) < abs(surface_normals(3, 1))
    v = [1, 0, 0];
elseif abs(surface_normals(2, 1)) < abs(surface_normals(1, 1)) && abs(surface_normals(2, 1)) < abs(surface_normals(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(surface_normals(:, 1), v);
u = u / norm(u);
v = cross(surface_normals(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
circle1 = R * (u' * cos(theta) + v' * sin(theta));
plot3(circle_centers(1, 1) + circle1(1, :), circle_centers(2, 1) + circle1(2, :), circle_centers(3, 1) + circle1(3, :), 'b', 'LineWidth', 4);hold on;

%second circle
if abs(surface_normals(1, 2)) < abs(surface_normals(2, 2)) && abs(surface_normals(1, 2)) < abs(surface_normals(3, 2))
    v = [1, 0, 0];
elseif abs(surface_normals(2, 2)) < abs(surface_normals(1, 2)) && abs(surface_normals(2, 2)) < abs(surface_normals(3, 2))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(surface_normals(:, 2), v);
u = u / norm(u);
v = cross(surface_normals(:, 2), u);
theta = linspace(0, 2*pi, sample_num);
circle2 = R * (u' * cos(theta) + v' * sin(theta));
plot3(circle_centers(1, 2) + circle2(1, :), circle_centers(2, 2) + circle2(2, :), circle_centers(3, 2) + circle2(3, :), 'r', 'LineWidth', 4); hold off;


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
%Initial Circle map has two candidate
InitialMap1 = [circle_centers(1, 1) + circle1(1, :); circle_centers(2, 1) + circle1(2, :); circle_centers(3, 1) + circle1(3, :)];
InitialMap2 = [circle_centers(1, 2) + circle2(1, :); circle_centers(2, 2) + circle2(2, :); circle_centers(3, 2) + circle2(3, :)]; 
%일단 대강으로 ellipse cone ray와 두 circle point간의 최댄거리인 ray와 그 3D circle point를
%연결짓자
% for i = 1:sample_num %map point
%     distance1 = zeros(1, sample_num);
%     distance2 = zeros(1, sample_num);
%     for j = 1:sample_num %rays
%         P1 = InitialMap1(:, i);
%         P2 = InitialMap2(:, i);
%         line_point = [0; 0; 0];
%         line_direction = [rays(1, j); rays(2, j); rays(3, j)];
%         P1_a = P1 - line_point;
%         P2_a = P2 - line_point;
%         cross_product1 = cross(line_direction, P1_a);
%         cross_product2 = cross(line_direction, P2_a);
%         distance1(j) = norm(cross_product1) / norm(line_direction);
%         distance2(j) = norm(cross_product2) / norm(line_direction);
%     end
%     [min_distance1(i), min_index1(i)] = min(distance1);
%     [min_distance2(i), min_index2(i)] = min(distance2); %i번째 map point에 해당하는 ray는 j = min_index
% end


%% Reproject test with true pose
% ransac_number = 1000;
% for i = 1:ransac_number
% 
% end
next = load('data/circle61.mat', 'C','imagePath', 'T_iss_cam', 'ellipses_params');
figure_detected_ellipse(next.imagePath, next.ellipses_params);
T1_inv = inv(T_iss_cam);
T_relative = T1_inv * next.T_iss_cam;
T_relative = inv(T_relative);
Rk = T_relative(1:3, 1:3);
Tk = T_relative(1:3, 4);

% 1st map
% Transform 3D points to the camera coordinate system of the next frame
for i = 1:sample_num
    P_camera = Rk * InitialMap1(:, i) + Tk;
    
    % Convert to homogeneous coordinates by appending a row of ones
    P_camera_homogeneous = P_camera;
    
    % Project 3D points to 2D using intrinsic matrix
    projected_points = intrinsics.K * P_camera_homogeneous;
    
    % Convert from homogeneous to 2D coordinates
    projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    
    % Extract the 2D points
    u1(i) = projected_points(1, :);
    v1(i) = projected_points(2, :);

    %reprojection error of ellipse constraint
    e1(i) = abs([u1(i) v1(i) 1]* next.C * [u1(i); v1(i); 1]);
end
%second map
for i = 1:sample_num
    P_camera = Rk * InitialMap2(:, i) + Tk;
    
    % Convert to homogeneous coordinates by appending a row of ones
    P_camera_homogeneous = P_camera;
    
    % Project 3D points to 2D using intrinsic matrix
    projected_points = intrinsics.K * P_camera_homogeneous;
    
    % Convert from homogeneous to 2D coordinates
    projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    
    % Extract the 2D points
    u2(i) = projected_points(1, :);
    v2(i) = projected_points(2, :);

    e2(i) = abs([u2(i) v2(i) 1]* next.C * [u2(i); v2(i); 1]);
end
% Visualization

img = imread(next.imagePath);
figure;
imshow(img); 
hold on;
plot(u1, v1, 'bo', 'MarkerSize',1); hold on;% Plot reprojected points in red
plot(u2, v2, 'ro', 'MarkerSize',1); % Plot reprojected points in red
legend('First Circle', 'Second Circle');
title('Reprojected 2D Points on Image Plane');
hold off;



%% Check the real solution
% show both solution
[p_iss_circle1, s_iss_circle1] = convert_cam_to_iss_frame(T_iss_cam, circle_centers(:, 1), surface_normals(:, 1));
[p_iss_circle2, s_iss_circle2] = convert_cam_to_iss_frame(T_iss_cam, circle_centers(:, 2), surface_normals(:, 2));
pm_normal = [0, 1, 0];
pm_position = [10.9349; -10.5572; 5.2508];
if dot(s_iss_circle1, pm_normal) > dot(s_iss_circle2, pm_normal)
    s_iss_circle = s_iss_circle1;
    p_iss_circle = p_iss_circle1;
else
    s_iss_circle = s_iss_circle2;
    p_iss_circle = p_iss_circle2;
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

if abs(s_iss_circle1(1)) < abs(s_iss_circle1(2)) && abs(s_iss_circle1(1)) < abs(s_iss_circle1(3))
    v = [1, 0, 0];
elseif abs(s_iss_circle1(2)) < abs(s_iss_circle1(1)) && abs(s_iss_circle1(2)) < abs(s_iss_circle1(3))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(s_iss_circle1, v);
u = u / norm(u);
v = cross(s_iss_circle1, u);
theta = linspace(0, 2*pi, 100);
circle1 = R * (u' * cos(theta) + v' * sin(theta));
plot3(p_iss_circle1(1) + circle1(1, :), p_iss_circle1(2) + circle1(2, :), p_iss_circle1(3) + circle1(3, :), 'b', 'LineWidth', 2);hold on;

if abs(s_iss_circle2(1)) < abs(s_iss_circle2(2)) && abs(s_iss_circle2(1)) < abs(s_iss_circle2(3))
    v = [1, 0, 0];
elseif abs(s_iss_circle2(2)) < abs(s_iss_circle2(1)) && abs(s_iss_circle2(2)) < abs(s_iss_circle2(3))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(s_iss_circle2, v);
u = u / norm(u);
v = cross(s_iss_circle2, u);
theta = linspace(0, 2*pi, 100);
circle2 = R * (u' * cos(theta) + v' * sin(theta));
plot3(p_iss_circle2(1) + circle2(1, :), p_iss_circle2(2) + circle2(2, :), p_iss_circle2(3) + circle2(3, :), 'r', 'LineWidth', 2);hold on;

plot_camera_frame(R_iss_cam, t_iss_cam, 1, 'k'); hold on;
plot_inertial_frame(1);
plot3(p_iss_circle1(1), p_iss_circle1(2), p_iss_circle1(3), 'bo'); 
plot3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), 'ro'); 

quiver3(p_iss_circle1(1), p_iss_circle1(2), p_iss_circle1(3), s_iss_circle1(1), s_iss_circle1(2), s_iss_circle1(3), 'b', 'LineWidth', 4);
quiver3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), s_iss_circle2(1), s_iss_circle2(2), s_iss_circle2(3), 'r', 'LineWidth', 4); 
quiver3(p_iss_circle(1), p_iss_circle(2), p_iss_circle(3), s_iss_circle(1), s_iss_circle(2), s_iss_circle(3), 'm', 'LineWidth', 1); hold off;


xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([10.2 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame');

f = FigureRotator();
%% Report
%reprojection error
disp('Reprojection error report');
if(sum(e1) < sum(e2))
    fprintf("Optimal Circle Solution is 1st\n");
else
    fprintf("Optimal Circle Solution is 2nd\n");
end

%true
disp('True Report');
if dot(s_iss_circle1, pm_normal) > dot(s_iss_circle2, pm_normal)
    fprintf("Optimal Circle Solution is 1st (blue)\n");
else
    fprintf("Optimal Circle Solution is 2nd (red)\n");
end

%% Optimization
next.bbox = [442, 873, 141, 570]; %y1 y2 x1 x2
Qe = next.C;
bbox = next.bbox;
T_last_inv = inv(T_iss_cam);
T_relative = T_last_inv * next.T_iss_cam;
T_relative = inv(T_relative);
R_true = T_relative(1:3, 1:3);
T_true = T_relative(1:3, 4);
% R_init = eye(3);
% T_init = [0;0;0];
R_init = R_true*[0.99 0 0; 0 0.99 0; 0 0 0.99];
T_init = T_true+[0.1; -0.1; -0.1];
%optimization per frame
objective_function = @(params) EReprojection(params, InitialMap2, Qe, intrinsics.K);
nonlinear_constraint = @(params) nonlcon(params, InitialMap2, intrinsics.K, bbox);
initial_params = [R_init, T_init];

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
[opt_params, ~] = lsqnonlin(objective_function, initial_params, [], [], options);

% options = optimoptions('fmincon', 'Algorithm', 'interior-point');
% [opt_params, ~] = fmincon(objective_function, initial_params, [], [], [], [], [], [], nonlinear_constraint, options);

R_relative_opt= opt_params(1:3, 1:3);
T_relative_opt = opt_params(1:3, 4);
Relative_Trans_opt = [R_relative_opt, T_relative_opt; 0 0 0 1];

EstimatedPose = T_iss_cam * Relative_Trans_opt;

%visualize reprojection with optimized R,t
for i = 1:sample_num
    P_camera = R_relative_opt * InitialMap2(:, i) + T_relative_opt;
    
    % Convert to homogeneous coordinates by appending a row of ones
    P_camera_homogeneous = P_camera;
    
    % Project 3D points to 2D using intrinsic matrix
    projected_points = intrinsics.K * P_camera_homogeneous;
    
    % Convert from homogeneous to 2D coordinates
    projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
    projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
    
    % Extract the 2D points
    u(i) = projected_points(1, :);
    v(i) = projected_points(2, :);

    e(i) = abs([u(i) v(i) 1]* next.C * [u(i); v(i); 1]);
end
% Visualization

img = imread(next.imagePath);
figure;
imshow(img); 
hold on;
plot(u, v, 'mo', 'MarkerSize',1); hold on;% Plot reprojected points in red
% plot(u2, v2, 'ro', 'MarkerSize',1); % Plot reprojected points in red
legend('First Circle', 'Second Circle');
title('Reprojected 2D Points on Image Plane with optimized R T');
hold off;

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

function [c, ceq] = nonlcon(params, Point3D, K, bbox)
    ceq = [];
    Rk = params(1:3, 1:3);
    Tk = params(1:3, 4);
    n = size(Point3D, 2);

    c = zeros(4*n, 1); % Initialize constraints array
    for i = 1:n
        % 3D point
        point3D = Point3D(:, i);
        projected_point = K * (Rk * point3D + Tk);
        projected_point = projected_point(1:2) / projected_point(3);
        
        % Constraints
        c(4*(i-1)+1) = bbox(3) - projected_point(1); % x >= bbox(3)
        c(4*(i-1)+2) = projected_point(1) - bbox(4); % x <= bbox(4)
        c(4*(i-1)+3) = bbox(1) - projected_point(2); % y >= bbox(1)
        c(4*(i-1)+4) = projected_point(2) - bbox(2); % y <= bbox(2)
    end
end