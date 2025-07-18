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
% ellipses_params = [644.2, 498.46, 352.73, 321.16, -49.485, 0.8]; %56
% ellipses_params = [689.39, 180.65, 476.66, 369.89, -64.619, 0.78]; %67
% R = [0.6, 0.15]; % real hatch radius
R = [0.5455 0.145];
visualize_colmap = 1; % optional. 

focalLength    = [608.210845 608.210845]; 
principalPoint = [640 440];
imageSize      = [1280 880];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

datasetPath = 'data/astrobee_colmap_groundtruth_ISS_forward/';

%61
% imagePath = 'data/000061.png'; % Replace with your image path
% ellipses_params = [6.58133728e+02, 3.66110321e+02, 4.00997986e+02, 3.38540649e+02, -5.80949249e+01, 8.33195984e-01;
%                     5.51632751e+02, 3.85255890e+02, 9.69921494e+01, 8.70498047e+01, -6.58237381e+01, 9.76479292e-01];
% P_iss_cam = [10.7428372221019988 -7.8855939000384510 4.6029741614027033 0.6872709057144766 0.1553442272110560 0.1777438272209571 0.6869745301795764]; % groundtruth pose 61
%56
% imagePath = 'data/000056.png';
% ellipses_params = [6.4420367e+02, 4.9845639e+02, 3.5272824e+02, 3.2116113e+02, -4.9485401e+01, 8.7966257e-01;
%                     5.4486017e+02, 5.0792032e+02, 8.6486214e+01, 8.2248230e+01, -5.4526070e+01, 9.7721386e-01];
% P_iss_cam = [10.7185919753656673 -7.8920022893684916 4.6034887030573968 0.7000022508165437 0.0966271102080742 0.1171396957510567 0.6978096746993121]; %groundtruth 56

%ff_forward occluded
imagePath = 'data/ff_return_journey_forward/gray/1617290608383992576.png';
ellipses_params = [8.4826917e+02  6.7294269e+02  5.9775848e+02  5.6279254e+02 1.2716424e+01  8.7090272e-01;
                    6.6501221e+02  6.7049939e+02  1.4285074e+02  1.3732433e+02 1.5479026e+01  9.7453570e-01];
P_iss_cam = [10.9001703876851455 -9.1295568986388655 4.5330361486412256 0.7519008943152273 0.0007513924664499 -0.0228556861474494 0.6588794261077300]; %groundtruth 56

% wrongly detected ellipse
% imagePath = 'data/td_yaw4/gray_aamed/1563961558217700864.png';
% ellipses_params = [641.3531494140625 567.8675537109375 358.579833984375 330.0566101074219 -66.39795684814453;
%             543.0610961914062 561.7825317382812 83.78350067138672 81.00579833984375 -33.070220947265625];
% P_iss_cam = [10.7113460747056095 -7.8949157867186992 4.6034067889120447 0.7043396039439703 0.0678306477919004 0.0881714072930431 0.7010923822665215];

% td yaw
% imagePath = 'data/td_yaw/gray/1563961557211092480.png';
% ellipses_params = [641.1278076171875 671.1201782226562 313.63885498046875 306.1940002441406 -10.105938911437988 0;
%                     541.1180419921875 669.3828125 81.4900131225586 77.96859741210938 41.51372146606445 1];
% P_iss_cam = [10.6922090272746324 -7.8947773282982476 4.6044744192759675 0.7093501503685319 0.0097194581137963 0.0299221186021887 0.7041537922389340];


%ff_forward
% imagePath = 'data/ff_return_journey_forward/gray/1617290513673136128.png';
% ellipses_params = [542.4254760742188 652.1347045898438 153.884765625 153.37484741210938 15.045231819152832 0;
%                     496.86566162109375 652.5275268554688 38.48677444458008 38.459110260009766 73.4066162109375 1];
% P_iss_cam = [10.8760041653805715 -5.5388859425793546 4.5716990302447220 0.7196230707482600 0.0068965888326207 -0.0140177303376216 0.6941891502648974];


% iva kibo rot
% imagePath = 'data/iva_kibo_rot/gray/1652460949373554176.png';
% ellipses_params = [490.5162658691406 775.0311889648438 734.1002197265625 694.97021484375 31.023462295532227 0;
%                 263.9907531738281 779.6212768554688 175.50277709960938 161.62399291992188 57.424678802490234 1];
% P_iss_cam = [10.5767293686300157 -9.6090663975677746 5.3418826454020456 0.7527088744182174 0.0465260324616480 0.0257060646183487 0.6562041427157217];

% ff_return_journey_ 뭔가 이상한거 일관적이지 않지만 잘 디텍드된것
% imagePath = 'data/ff_return_journey_forward/gray/1617290525285450496.png';
% ellipses_params = [542.5059204101562 641.2716674804688 168.26866149902344 167.05958557128906 -43.61550521850586 0;
%                     493.21807861328125 642.0353393554688 41.950382232666016 41.80957794189453 -58.30564498901367 1];
% P_iss_cam = [10.8849017992489507 -5.9655100406562482 4.5954148884120096 0.7175399324260420 0.0158795299919192 -0.0120941653979192 0.6962312956658833];

R_iss_cam = quat2rotm([P_iss_cam(7), P_iss_cam(4:6)]);
t_iss_cam = P_iss_cam(1:3)';
T_iss_cam = [R_iss_cam, t_iss_cam; 0 0 0 1];

%% Ellipse equation
circle_centers = cell(1, 2);
surface_normals = cell(1, 2);
C_quad = cell(1, 2);
for i = 1:size(ellipses_params, 1)
    k = ellipses_params(i, 1); %y
    h = ellipses_params(i, 2); %x
    a = ellipses_params(i, 3)/2; %a
    b = ellipses_params(i, 4)/2; %b
    angle = 90-ellipses_params(i, 5); %angle (deg)
    % ellipse equation to quadratic form 
    [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
    C_quad{i} = [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F]; 

%% Cone Equation
    Q = intrinsics.K'* C_quad{i} * intrinsics.K;
    
    % eigen value and normalized eigen vector of Q
    [V, D] = eig(Q); % eigen value D, eigen vector V
    % Normalize the eigenvectors
    for j = 1:size(V, 2)
        V(:, j) = V(:, j) / norm(V(:, j));
    end
    
    [U, lam] = find_transform_cone_cam(V, D);
    
    % circle 3d position
    % circle center has two solution
    circle_centers{i} = find_circle_center_position(U, lam, R(i));
    
    % surface normal has two solution corresponding 
    surface_normals{i} = find_circle_surface_normal(U, lam);

end

%Choose the right circles
dotProducts = [
    dot(surface_normals{1}(:, 1), surface_normals{2}(:, 1))
    dot(surface_normals{1}(:, 2), surface_normals{2}(:, 1))
    dot(surface_normals{1}(:, 1), surface_normals{2}(:, 2))
    dot(surface_normals{1}(:, 2), surface_normals{2}(:, 2))
];

[maxDotProduct, maxIndex] = max(dotProducts);
angle_diff = acos(maxDotProduct)*180/pi;
fprintf("Angle Difference between 2 Circles : %f", angle_diff);

switch maxIndex
    case 1
        PM_normal_cam = surface_normals{1}(:, 1);
        PM_center_cam = circle_centers{1}(:, 1);
        Window_normal_cam = surface_normals{2}(:, 1);
        Window_center_cam = circle_centers{2}(:, 1);
    case 2
        PM_normal_cam = surface_normals{1}(:, 2);
        PM_center_cam = circle_centers{1}(:, 2);
        Window_normal_cam = surface_normals{2}(:, 1);
        Window_center_cam = circle_centers{2}(:, 1);
    case 3
        PM_normal_cam = surface_normals{1}(:, 1);
        PM_center_cam = circle_centers{1}(:, 1);
        Window_normal_cam = surface_normals{2}(:, 2);
        Window_center_cam = circle_centers{2}(:, 2);
    case 4
        PM_normal_cam = surface_normals{1}(:, 2);
        PM_center_cam = circle_centers{1}(:, 2);
        Window_normal_cam = surface_normals{2}(:, 2);
        Window_center_cam = circle_centers{2}(:, 2);
end

[PM_center_iss, PM_normal_iss] = convert_cam_to_iss_frame(T_iss_cam, PM_center_cam, PM_normal_cam);
[Window_center_iss, Window_normal_iss] = convert_cam_to_iss_frame(T_iss_cam, Window_center_cam, Window_normal_cam);

save data/circle_wrong_p2c.mat
% ================= Visualize Results =================
pm_normal = [0, 1, 0];
pm_position = [10.9349; -10.5572; 5.2508];
win_normal = [0, 1, 0];
win_position = [10.9349; -10.5572; 4.88];
% draw results
s_iss_circle = cell(1, 2);
p_iss_circle = cell(1, 2);
p_iss_circle1 = cell(1, 2);
s_iss_circle2 = cell(1, 2);
figure_detected_two_ellipse(imagePath, ellipses_params, imagePath);    
for i = 1:2
    [p_iss_circle1{i}, s_iss_circle1{i}] = convert_cam_to_iss_frame(T_iss_cam, circle_centers{i}(:, 1), surface_normals{i}(:, 1));
    [p_iss_circle2{i}, s_iss_circle2{i}] = convert_cam_to_iss_frame(T_iss_cam, circle_centers{i}(:, 2), surface_normals{i}(:, 2));
    if dot(s_iss_circle1{i}, pm_normal) > dot(s_iss_circle2{i}, pm_normal)
        s_iss_circle{i} = s_iss_circle1{i};
        p_iss_circle{i} = p_iss_circle1{i};
    else
        s_iss_circle{i} = s_iss_circle2{i};
        p_iss_circle{i} = p_iss_circle2{i};
    end

    figure_3dcircles_camera_frame(circle_centers{i}, surface_normals{i});
end

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

plot_camera_frame(R_iss_cam, t_iss_cam, 1, 'k'); hold on;
plot_inertial_frame(1);
for i = 1:2
    plot3(p_iss_circle1{i}(1), p_iss_circle1{i}(2), p_iss_circle1{i}(3), 'bo'); hold on
    plot3(p_iss_circle2{i}(1), p_iss_circle2{i}(2), p_iss_circle2{i}(3), 'ro'); 
    
    quiver3(p_iss_circle1{i}(1), p_iss_circle1{i}(2), p_iss_circle1{i}(3), s_iss_circle1{i}(1), s_iss_circle1{i}(2), s_iss_circle1{i}(3), 'b', 'LineWidth', 4);
    quiver3(p_iss_circle2{i}(1), p_iss_circle2{i}(2), p_iss_circle2{i}(3), s_iss_circle2{i}(1), s_iss_circle2{i}(2), s_iss_circle2{i}(3), 'r', 'LineWidth', 4); 
end

xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -5]);
zlim([4 10]);
xlim([10.2 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame');
hold off
f = FigureRotator();


%% Visualize the solution calculated from dot product


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

plot_camera_frame(R_iss_cam, t_iss_cam, 1, 'k'); hold on;
plot_inertial_frame(1);
plot3(PM_center_iss(1), PM_center_iss(2), PM_center_iss(3), 'bo'); hold on
plot3(Window_center_iss(1), Window_center_iss(2), Window_center_iss(3), 'ro'); 

quiver3(PM_center_iss(1), PM_center_iss(2), PM_center_iss(3), PM_normal_iss(1), PM_normal_iss(2), PM_normal_iss(3), 'b', 'LineWidth', 4); 
quiver3(Window_center_iss(1), Window_center_iss(2), Window_center_iss(3), Window_normal_iss(1), Window_normal_iss(2), Window_normal_iss(3), 'r', 'LineWidth', 4);

xlabel('X'); ylabel('Y'); zlabel('Z');
ylim([-15 -6.5]);
zlim([4 10]);
xlim([10.2 14]);
grid on;
axis equal;
title('3D Visualization w.r.t iss world frame');
hold off
f = FigureRotator();

