clc;
close all;
clear variables; %clear classes;

%%
load('data/circle56.mat');
sample_num = 1000;
%% Backproject Corresponding pixel from 2D ellipse to 3D space

% Pick sample points in 2D ellipse
img = imread(imagePath); 
theta = deg2rad(angle);
t = linspace(0, 2*pi, sample_num); % Sample Points
X = a * cos(t);
Y = b * sin(t);
ellipse_orientation = [cos(theta), -sin(theta); sin(theta), cos(theta)];
ellipse_points = ellipse_orientation * [X; Y];
sample_x = ellipse_points(1, :) + h;
sample_y = ellipse_points(2, :) + k;
figure;
imshow(img); hold on;
plot(sample_x, sample_y, 'ro'); % Use plot for 2D plotting
title('Sample Points in 2D Ellipse');


k = ellipses_params(1); %y
h = ellipses_params(2); %x
a = ellipses_params(3)/2; %a
b = ellipses_params(4)/2; %b
theta = 90-ellipses_params(5); %angle (deg)
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
% plot3(circle_centers(1, 1) + circle1(1, :), circle_centers(2, 1) + circle1(2, :), circle_centers(3, 1) + circle1(3, :), 'k', 'LineWidth', 4);hold on;

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
% plot3(circle_centers(1, 2) + circle2(1, :), circle_centers(2, 2) + circle2(2, :), circle_centers(3, 2) + circle2(3, :), 'k', 'LineWidth', 4);hold on;


%% Circle only motion tracking
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

% Test Correspondence Visualization
test_point = 70;
figure;
plot_inertial_frame(1); hold on; % Camera Frame
k = ellipses_params(1); %y
h = ellipses_params(2); %x
a = ellipses_params(3)/2; %a
b = ellipses_params(4)/2; %b
theta = 90-ellipses_params(5); %angle (deg)
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
% for i = 1:num_points
%     plot3([0 rays(1, i)], ...
%           [0 rays(2, i)], ...
%           [0 rays(3, i)], 'b-');
% end
% Plot the ellipse points on the image plane (z = 1)
plot3(x_cam, y_cam, ones(1, num_points), 'ro');
plot3(x_cam(min_index1(test_point)), y_cam(min_index1(test_point)), 1, 'bo', 'MarkerSize', 5);

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
plot3(circle_centers(1, 1) + circle1(1, :), circle_centers(2, 1) + circle1(2, :), circle_centers(3, 1) + circle1(3, :), 'k', 'LineWidth', 1);hold on;

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
plot3(circle_centers(1, 2) + circle2(1, :), circle_centers(2, 2) + circle2(2, :), circle_centers(3, 2) + circle2(3, :), 'k', 'LineWidth', 1);hold on;

% Plot the rays and corresponding point
plot3([0 rays(1, min_index1(test_point))], ...
      [0 rays(2, min_index1(test_point))], ...
      [0 rays(3, min_index1(test_point))], 'r-');
plot3(InitialMap1(1, test_point), InitialMap1(2, test_point), InitialMap1(3, test_point), 'ro', 'MarkerSize', 5); hold off;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
% ylim([-10 10]);
% zlim([-10 10]);
% xlim([-10 10]);
title('Test corresponding ray');
f = FigureRotator();


%% Reproject to next frame
% k 번째 MapPoint에 해당하는 ray는 min_index(k) 이고, 그에 해당하는 2D ellipse point도
% ellipse min_index(k) 인데 이것들의 순서를 map point에 맞추자!
mapPoints = InitialMap1;
for i = 1:sample_num
    imagePoints = [x_ellipse(min_index1(i)); y_ellipse(min_index1(i))];
end
%두번째 이미지 ellipse detect해서 ellipse param구해와서 Qellipse에 넣어
next = load('data/circle61.mat', 'C');
Qellipse = next.C;


Rk = eye(3); % Initial rotation matrix
tk = zeros(3, 1); % Initial translation vector

% Projection function
project = @(Rk, tk, P) (Rk * P + tk); % @(변수) (함수)


% Huber cost function
huber = @(r, delta) (abs(r) <= delta) .* (0.5 * r.^2) + (abs(r) > delta) .* (delta * (abs(r) - 0.5 * delta));

% Residual function
residual = @(params, InitialMap1, imagePoints, Qellipse) ...
    arrayfun(@(i) norm( ...
        huber( ...
            project(reshape(params(1:9), [3, 3]), params(10:12), mapPoints(:,i))' * Qellipse * ...
            project(reshape(params(1:9), [3, 3]), params(10:12), mapPoints(:,i)) - imagePoints(:,i), ...
            1)), ...
        1:size(mapPoints,2))';

% Levenberg-Marquardt optimization
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt');
params0 = [Rk(:); tk];
params_opt = lsqnonlin(@(params) residual(params, mapPoints, imagePoints, Qellipse), params0, [], [], opts);

% Extract optimized R and t
R_opt = reshape(params_opt(1:9), [3, 3]);
t_opt = params_opt(10:12);

% Display results
disp('Optimized Rotation Matrix:');
disp(R_opt);
disp('Optimized Translation Vector:');
disp(t_opt);

% Visualization of projected points
figure;
hold on;
plot(imagePoints(1,:), imagePoints(2,:), 'ro'); % Original 2D points
projectedPoints = arrayfun(@(i) project(R_opt, t_opt, mapPoints(:,i)), 1:size(mapPoints,2), 'UniformOutput', false);
projectedPoints = cell2mat(projectedPoints);
plot(projectedPoints(1,:), projectedPoints(2,:), 'bx'); % Projected 2D points
legend('Original 2D Points', 'Projected 2D Points');
title('Original vs Projected 2D Points');
hold off;