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


%% Input Params Setting
% ellipses_params = [8.66740417e+02, 6.74527893e+02, 5.90378113e+02, 5.43745300e+02, 1.80407467e+01, 8.99021149e-01]; % modify to your ellipse (ycenter, xcenter, major length, minor length, theta, eccentity)
% ellipses_params = [6.58133728e+02, 3.66110321e+02, 4.00997986e+02, 3.38540649e+02, -5.80949249e+01, 8.33195984e-01]; %61
% ellipses_params = [5.51632751e+02, 3.85255890e+02, 9.69921494e+01, 8.70498047e+01, -6.58237381e+01, 9.76479292e-01]; %61 window
% ellipses_params = [644.2, 498.46, 352.73, 321.16, -49.485, 0.8]; %56
% ellipses_params = [689.39, 180.65, 476.66, 369.89, -64.619, 0.78]; %67

R = 0.6; % real hatch radius
visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

% imagePath = 'data/000061.png'; % Replace with your image path
datasetPath = 'data/astrobee_colmap_groundtruth_ISS_forward/';

imagePath = 'data/ff_return_journey_forward/gray/1617290608383992576.png';
ellipses_params = [8.4826917e+02  6.7294269e+02  5.9775848e+02  5.6279254e+02 1.2716424e+01  8.7090272e-01];
P_iss_cam = [10.9001703876851455 -9.1295568986388655 4.5330361486412256 0.7519008943152273 0.0007513924664499 -0.0228556861474494 0.6588794261077300]; %groundtruth 56


% P_iss_cam = [10.7691269315059888 -7.8709607218059174 4.6082869961627182 0.6652476095285249 0.2245446562708415 0.2488764354501405 0.6671475363324251]; % groundtruth pose 67
% P_iss_cam = [10.7428372221019988 -7.8855939000384510 4.6029741614027033 0.6872709057144766 0.1553442272110560 0.1777438272209571 0.6869745301795764]; % groundtruth pose 61
% P_iss_cam = [10.7185919753656673 -7.8920022893684916 4.6034887030573968 0.7000022508165437 0.0966271102080742 0.1171396957510567 0.6978096746993121]; %groundtruth 56
% P_iss_cam = [10.9008255202805611 -9.1514648174879820 4.5299185383477134 0.7552648685451584 0.0019899022938505 -0.0176615630708548 0.6551786686242308];
R_iss_cam = quat2rotm([P_iss_cam(7), P_iss_cam(4:6)]);
t_iss_cam = P_iss_cam(1:3)';
T_iss_cam = [R_iss_cam, t_iss_cam; 0 0 0 1];




%% Ellipse equation
k = ellipses_params(1); %y
h = ellipses_params(2); %x
a = ellipses_params(3)/2; %a
b = ellipses_params(4)/2; %b
angle = 90-ellipses_params(5); %angle (deg)
% ellipse equation to quadratic form 
[A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
C = [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F]; 

%% Cone Equation
Q = intrinsics.K'* C * intrinsics.K;

% eigen value and normalized eigen vector of Q
[V, D] = eig(Q); % eigen value D, eigen vector V
% Normalize the eigenvectors
for i = 1:size(V, 2)
    V(:, i) = V(:, i) / norm(V(:, i));
end

[U, lam] = find_transform_cone_cam(V, D);

%% circle 3d position

% circle center has two solution
circle_centers = find_circle_center_position(U, lam, R);

% surface normal has two solution corresponding 
surface_normals = find_circle_surface_normal(U, lam);


%% ================= Visualize Results =================

%% draw results
figure_detected_ellipse(imagePath, ellipses_params);
figure_3dcircles_camera_frame(circle_centers, surface_normals);

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

save data/circle67.mat

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
plot3(p_iss_circle1(1) + circle1(1, :), p_iss_circle1(2) + circle1(2, :), p_iss_circle1(3) + circle1(3, :), 'r', 'LineWidth', 2);hold on;

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
plot3(p_iss_circle2(1) + circle2(1, :), p_iss_circle2(2) + circle2(2, :), p_iss_circle2(3) + circle2(3, :), 'b', 'LineWidth', 2);hold on;

plot_camera_frame(R_iss_cam, t_iss_cam, 1, 'k'); hold on;
plot_inertial_frame(1);
plot3(p_iss_circle1(1), p_iss_circle1(2), p_iss_circle1(3), 'ro'); 
plot3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), 'bo'); 

quiver3(p_iss_circle1(1), p_iss_circle1(2), p_iss_circle1(3), s_iss_circle1(1), s_iss_circle1(2), s_iss_circle1(3), 'r', 'LineWidth', 4);
quiver3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), s_iss_circle2(1), s_iss_circle2(2), s_iss_circle2(3), 'b', 'LineWidth', 4); hold off;


xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([10.2 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame');

f = FigureRotator();

% show one solution
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

if abs(s_iss_circle(1)) < abs(s_iss_circle(2)) && abs(s_iss_circle(1)) < abs(s_iss_circle(3))
    v = [1, 0, 0];
elseif abs(s_iss_circle(2)) < abs(s_iss_circle(1)) && abs(s_iss_circle(2)) < abs(s_iss_circle(3))
    v = [0, 1, 0];
else
    v = [0, 0, 1];
end
u = cross(s_iss_circle, v);
u = u / norm(u);
v = cross(s_iss_circle, u);
theta = linspace(0, 2*pi, 100);
circle1 = R * (u' * cos(theta) + v' * sin(theta));
plot3(p_iss_circle(1) + circle1(1, :), p_iss_circle(2) + circle1(2, :), p_iss_circle(3) + circle1(3, :), 'b', 'LineWidth', 2);hold on;

plot_camera_frame(R_iss_cam, t_iss_cam, 1, 'k'); hold on;
plot_inertial_frame(1);
plot3(p_iss_circle(1), p_iss_circle(2), p_iss_circle(3), 'ro'); 
% plot3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), 'ro'); 

quiver3(p_iss_circle(1), p_iss_circle(2), p_iss_circle(3), s_iss_circle(1), s_iss_circle(2), s_iss_circle(3), 'r', 'LineWidth', 4);
% quiver3(p_iss_circle2(1), p_iss_circle2(2), p_iss_circle2(3), s_iss_circle2(1), s_iss_circle2(2), s_iss_circle2(3), 'r'); hold off;


xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([10.5 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame');

f = FigureRotator();

