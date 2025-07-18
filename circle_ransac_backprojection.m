clc;
close all;
clear variables; %clear classes;

%%
load('data/circle56.mat');
next = load('data/circle61.mat');
sample_num = 1000;
view_num = 100;
firstC = 'b';
secondC = 'r';

%% Check Both Solution Given True Pose
figure;
% plot_inertial_frame(1); hold on; % Camera Frame
% draw the camera frame
camFrame = eye(3);
p_gc = [0;0;0];
line([camFrame(1,1) p_gc(1)],[camFrame(2,1) p_gc(2)],[camFrame(3,1) p_gc(3)],'Color','r','LineWidth',5); hold on; % x axis hold on;
line([camFrame(1,2) p_gc(1)],[camFrame(2,2) p_gc(2)],[camFrame(3,2) p_gc(3)],'Color','g','LineWidth',5); % y axis
line([camFrame(1,3) p_gc(1)],[camFrame(2,3) p_gc(2)],[camFrame(3,3) p_gc(3)],'Color','b','LineWidth',5); % z axis


k = ellipses_params(1); %y
h = ellipses_params(2); %x
a = ellipses_params(3)/2; %a
b = ellipses_params(4)/2; %b
theta = 90-ellipses_params(5); %angle (deg)
t = linspace(0, 2*pi, view_num);
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
          [0 rays(3, i)], 'c-');
end
% Plot the ellipse points on the image plane (z = 1)
plot3(x_cam, y_cam, ones(1, num_points), 'ro');

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
plot3(circle_centers(1, 1) + circle1(1, :), circle_centers(2, 1) + circle1(2, :), circle_centers(3, 1) + circle1(3, :), firstC, 'LineWidth', 4);hold on;

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
plot3(circle_centers(1, 2) + circle2(1, :), circle_centers(2, 2) + circle2(2, :), circle_centers(3, 2) + circle2(3, :), secondC, 'LineWidth', 4);hold on;


%Second Frame
T1_inv = inv(T_iss_cam);
T_relative = T1_inv * next.T_iss_cam;
% T_relative = inv(T_relative);
% T_relative = eye(4);
camFrame = T_relative(1:3, 1:3);
p_gc = T_relative(1:3, 4);
line([camFrame(1,1) p_gc(1)],[camFrame(2,1) p_gc(2)],[camFrame(3,1) p_gc(3)],'Color','r','LineWidth',5); % x axis
line([camFrame(1,2) p_gc(1)],[camFrame(2,2) p_gc(2)],[camFrame(3,2) p_gc(3)],'Color','g','LineWidth',5); % y axis
line([camFrame(1,3) p_gc(1)],[camFrame(2,3) p_gc(2)],[camFrame(3,3) p_gc(3)],'Color','b','LineWidth',5); % z axis


k = next.ellipses_params(1); %y
h = next.ellipses_params(2); %x
a = next.ellipses_params(3)/2; %a
b = next.ellipses_params(4)/2; %b
theta = 90-next.ellipses_params(5); %angle (deg)
t = linspace(0, 2*pi, view_num);
x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y_ellipse = k + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
x_cam = (x_ellipse - intrinsics.K(1, 3)) / intrinsics.K(1, 1);
y_cam = (y_ellipse - intrinsics.K(2, 3)) / intrinsics.K(2, 2);
z_cam = ones(1, sample_num);
% Define the rays (from camera center through each ellipse point)
num_points = length(x_cam);
rays = zeros(3, num_points);
scaling_factor = 10;
for i = 1:num_points
    % Each ray in normalized camera coordinates
    new_cam = T_relative(1:3, 1:3) * [x_cam(i); y_cam(i); z_cam(i) ] + T_relative(1:3, 4);
    x_cam(i) = new_cam(1);
    y_cam(i) = new_cam(2);
    z_cam(i) = new_cam(3);
    rays(:, i) = [x_cam(i); y_cam(i); z_cam(i)];
    % Normalize the ray
    rays(:, i) = rays(:, i) / norm(rays(:, i));
    rays(:, i) = scaling_factor * rays(:, i);
end
% Plot the rays
for i = 1:num_points
    plot3([T_relative(1, 4) rays(1, i)], ...
          [T_relative(2, 4) rays(2, i)], ...
          [T_relative(3, 4) rays(3, i)], 'm-');
end
% Plot the ellipse points on the image plane (z = 1)
plot3(x_cam, y_cam, ones(1, num_points), 'ro');

%first circle
surface_normals_next(1:3, 1) = T_relative(1:3, 1:3)*next.surface_normals(1:3, 1)+T_relative(1:3, 4);
surface_normals_next(1:3, 2) = T_relative(1:3, 1:3)*next.surface_normals(1:3, 2)+T_relative(1:3, 4);
circle_centers_next(1:3, 1) = T_relative(1:3, 1:3)*next.circle_centers(1:3, 1)+T_relative(1:3, 4);
circle_centers_next(1:3, 2) = T_relative(1:3, 1:3)*next.circle_centers(1:3, 2)+T_relative(1:3, 4);
if abs(surface_normals_next(1, 1)) < abs(surface_normals_next(2, 1)) && abs(surface_normals_next(1, 1)) < abs(surface_normals_next(3, 1))
    v = [1, 0, 0];
elseif abs(surface_normals_next(2, 1)) < abs(surface_normals_next(1, 1)) && abs(surface_normals_next(2, 1)) < abs(surface_normals_next(3, 1))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(surface_normals_next(:, 1), v);
u = u / norm(u);
v = cross(surface_normals_next(:, 1), u);
theta = linspace(0, 2*pi, sample_num);
circle1_next = R * (u' * cos(theta) + v' * sin(theta));
plot3(circle_centers_next(1, 1) + circle1_next(1, :), circle_centers_next(2, 1) + circle1_next(2, :), circle_centers_next(3, 1) + circle1_next(3, :), firstC, 'LineWidth', 4);hold on;

%second circle
if abs(surface_normals_next(1, 2)) < abs(surface_normals_next(2, 2)) && abs(surface_normals_next(1, 2)) < abs(surface_normals_next(3, 2))
    v = [1, 0, 0];
elseif abs(surface_normals_next(2, 2)) < abs(surface_normals_next(1, 2)) && abs(surface_normals_next(2, 2)) < abs(surface_normals_next(3, 2))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(surface_normals_next(:, 2), v);
u = u / norm(u);
v = cross(surface_normals_next(:, 2), u);
theta = linspace(0, 2*pi, sample_num);
circle2_next = R * (u' * cos(theta) + v' * sin(theta));
plot3(circle_centers_next(1, 2) + circle2_next(1, :), circle_centers_next(2, 2) + circle2_next(2, :), circle_centers_next(3, 2) + circle2_next(3, :), secondC, 'LineWidth', 4);hold on;


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
plot3(x_cam, y_cam, ones(1, num_points), 'ro');

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
plot3(circle_centers(1, 1) + circle1(1, :), circle_centers(2, 1) + circle1(2, :), circle_centers(3, 1) + circle1(3, :), firstC, 'LineWidth', 4);hold on;

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
plot3(circle_centers(1, 2) + circle2(1, :), circle_centers(2, 2) + circle2(2, :), circle_centers(3, 2) + circle2(3, :), secondC, 'LineWidth', 4);hold on;


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
for i = 1:sample_num %map point
    distance1 = zeros(1, sample_num);
    distance2 = zeros(1, sample_num);
    for j = 1:sample_num %rays
        P1 = InitialMap1(:, i);
        P2 = InitialMap2(:, i);
        line_point = [0; 0; 0];
        line_direction = [rays(1, j); rays(2, j); rays(3, j)];
        P1_a = P1 - line_point;
        P2_a = P2 - line_point;
        cross_product1 = cross(line_direction, P1_a);
        cross_product2 = cross(line_direction, P2_a);
        distance1(j) = norm(cross_product1) / norm(line_direction);
        distance2(j) = norm(cross_product2) / norm(line_direction);
    end
    [min_distance1(i), min_index1(i)] = min(distance1);
    [min_distance2(i), min_index2(i)] = min(distance2); %i번째 map point에 해당하는 ray는 j = min_index
end


%% BackProjecto test with true pose

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


