clc;
close all;
clear variables; %clear classes;

%% preprocessing - only detected ellipse


imagePath = 'data/ff_return_journey_forward/gray/';
ellipsePath = 'data/ff_return_journey_forward/results_p2c/';
posePath = 'data/ff_return_journey_forward/groundtruth.txt';
% 
% imagePath = 'data/ff_return_journey_left/gray/';
% ellipsePath = 'data/ff_return_journey_left/results_p2c/';
% posePath = 'data/ff_return_journey_left/groundtruth.txt';

% imagePath = 'data/iva_kibo_rot/gray/';
% ellipsePath = 'data/iva_kibo_rot/results_p2c/';
% posePath = 'data/iva_kibo_rot/groundtruth.txt';

% imagePath = 'data/multiview_mixed_data/gray/';
% ellipsePath = 'data/multiview_mixed_data/results_p2c/';
% posePath = 'data/multiview_mixed_data/groundtruth.txt';

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

%%
true_pose = readmatrix(posePath, 'Delimiter', ' ');
for i = 1:size(ellipse_result1,1)
    matched = false;
    for j = 1:length(true_pose)
        if abs(ellipse_result1(i, 1) - true_pose(j, 1)) < 100000
            P_iss_cam_true(i, :) = true_pose(j, :);
            R_iss_cam_true{i} = quat2rotm([P_iss_cam_true(i, 8), P_iss_cam_true(i, 5:7)]);
            t_iss_cam_true{i} = P_iss_cam_true(i, 2:4)';
            T_iss_cam_true{i} = [R_iss_cam_true{i}, t_iss_cam_true{i}; 0 0 0 1];
            matched = true;
            break;
        end
    end
    if ~matched
        fprintf('[경고] Frame %d: 타임스탬프 매칭 실패\n', i);
        T_iss_cam_true{i} = []; % 명시적으로 빈 값 설정
    end
end


tracking_num = size(ellipse_result1, 1);
sample_num = 1000;

%% Input Params Setting
R = [0.5455, 0.145]; % real hatch radius
% R = [0.555 0.141];
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
angle_diffs = zeros(1, tracking_num);

for i = 1:tracking_num
    ellipse_params1 = ellipse_result1(i, 2:end);
    ellipse_params2 = ellipse_result2(i, 2:end);
    ellipses_params{i} = [ellipse_params1; ellipse_params2];
    
    [PM_normal_cam{i}, PM_center_cam{i}, Window_normal_cam{i}, Window_center_cam{i}, C_pm{i}, C_win{i}, angle_diff{i}] = perspective_two_circle(ellipses_params{i}, R, intrinsics.K);
    % fprintf("i=%d, angle_diff=%.2f\n", i, angle_diff{i});
    angle_diffs(i) = angle_diff{i};
    % Initial Ransac : Remove wrongly detected ellipse
    if angle_diff{i} > 3
        ransac(i) = 0;
    else
        if i == 1 
            tstampStr = sprintf('%.0f', tstamp(i));
            image = strcat(imagePath, imgList(i, :));
            figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
            % pause(4);
        end
        tstampStr = sprintf('%.0f', tstamp(i));
        image = strcat(imagePath, imgList(i, :));
        figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
        % pause(0.5);
    end

    z = ( PM_center_cam{i} - Window_center_cam{i} ) / norm(PM_center_cam{i} - Window_center_cam{i});
    x = cross(PM_normal_cam{i}, z);
    x = x/norm(x);
    z = cross(x, PM_normal_cam{i});
    y = PM_normal_cam{i};
    R_cam_p2c = [x, y, z];
    t_cam_p2c = PM_center_cam{i};
    T_cam_p2c{i} = [R_cam_p2c, t_cam_p2c; 0 0 0 1];
    % T_p2c_cam{i} = T_cam_p2c{1} * inv(T_cam_p2c{i});


    z = ( PM_center_cam{i} - Window_center_cam{i});
    z = z / norm(z);
    x = cross(Window_normal_cam{i}, z);
    x = x / norm(x);
    z = cross(x, Window_normal_cam{i});
    y = Window_normal_cam{i};
    R_cam_w2c = [x, y, z];
    t_cam_w2c = Window_center_cam{i};
    T_cam_w2c{i} = [R_cam_w2c, t_cam_w2c; 0 0 0 1];
    % T_w2c_cam{i} = T_cam_w2c{1} * inv(T_cam_w2c{i});

end




figure;
histogram(angle_diffs, 50);  % bin 개수는 필요에 따라 조절
xlabel('angle diff');
ylabel('Count');
title('Distribution of angle difference between PM and Window');
grid on;
fprintf('Angle diff statistics:\n');
fprintf('  Mean  = %.2f\n', mean(angle_diffs));
fprintf('  Std   = %.2f\n', std(angle_diffs));
fprintf('  Min   = %.2f\n', min(angle_diffs));
fprintf('  Max   = %.2f\n', max(angle_diffs));
disp(angle_diffs(8));

ransac = logical(ransac);
tstamp = tstamp(ransac);
imgList = imgList(ransac, :);
R_iss_cam_true = R_iss_cam_true(ransac);
t_iss_cam_true = t_iss_cam_true(ransac);
T_iss_cam_true = T_iss_cam_true(ransac);
% % T_p2c_cam = T_p2c_cam(ransac);
% T_w2c_cam = T_w2c_cam(ransac);
T_cam_p2c = T_cam_p2c(ransac);
T_cam_w2c = T_cam_w2c(ransac);
ellipses_params = ellipses_params(ransac);
PM_normal_cam = PM_normal_cam(ransac);
PM_center_cam = PM_center_cam(ransac);
Window_normal_cam = Window_normal_cam(ransac);
Window_center_cam = Window_center_cam(ransac);
C_pm = C_pm(ransac);
C_win = C_win(ransac);

tracking_num = size(R_iss_cam_true, 2);

% T_p2c_iss = T_p2c_cam{7} * inv(T_iss_cam_true{7});
% for i = 1:tracking_num
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end
% T_p2c_iss = T_p2c_cam{1} * inv(T_iss_cam_true{1});
% for i = 1:tracking_num
%     T_p2c_iss = T_p2c_cam{i} * inv(T_iss_cam_true{i});
%     T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
% end




%% Choose Initial Map Points with Cone Equation
% Visualize the cone
z_means = zeros(1, tracking_num);
InitialMap_PM_all = cell(1, tracking_num);
InitialMap_Win_all = cell(1, tracking_num);
valid_k_list = [];
valid_z_means = [];
for k=1:tracking_num
    % figure;
    % plot_inertial_frame(1); hold on; % Camera Frame
    for i = 1:2
        k1 = ellipses_params{k}(i, 1); %y
        h = ellipses_params{k}(i, 2); %x
        a = ellipses_params{k}(i, 3)/2; %a
        b = ellipses_params{k}(i, 4)/2; %b
        theta = 90-ellipses_params{k}(i, 5); %angle (deg)
        t = linspace(0, 2*pi, sample_num);
        x_ellipse = h + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
        y_ellipse = k1 + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);
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
        % % Plot the rays
        % for j = 1:40:num_points
        %     plot3([0 rays(1, j)], ...
        %         [0 rays(2, j)], ...
        %         [0 rays(3, j)], 'g-');
        % end
        % % Plot the ellipse points on the image plane (z = 1)
        % plot3(x_cam, y_cam, ones(1, num_points), 'mo');
    end

    %PM circle
    if abs(PM_normal_cam{k}(1, 1)) < abs(PM_normal_cam{k}(2, 1)) && abs(PM_normal_cam{k}(1, 1)) < abs(PM_normal_cam{k}(3, 1))
        v = [1, 0, 0];
    elseif abs(PM_normal_cam{k}(2, 1)) < abs(PM_normal_cam{k}(1, 1)) && abs(PM_normal_cam{k}(2, 1)) < abs(PM_normal_cam{k}(3, 1))
        v = [0, 1, 0];
    else
        v = [0, 0, 1];
    end
    u = cross(PM_normal_cam{k}(:, 1), v);
    u = u / norm(u);
    v = cross(PM_normal_cam{k}(:, 1), u);
    theta = linspace(0, 2*pi, sample_num);
    PM_circle_3D = R(1) * (u' * cos(theta) + v' * sin(theta));
    % plot3(PM_center_cam{1}(1, 1) + PM_circle_3D(1, :), PM_center_cam{1}(2, 1) + PM_circle_3D(2, :), PM_center_cam{1}(3, 1) + PM_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;

    %Window circle
    if abs(Window_normal_cam{k}(1, 1)) < abs(Window_normal_cam{k}(2, 1)) && abs(Window_normal_cam{k}(1, 1)) < abs(Window_normal_cam{k}(3, 1))
        v = [1, 0, 0];
    elseif abs(Window_normal_cam{k}(2, 1)) < abs(Window_normal_cam{k}(1, 1)) && abs(Window_normal_cam{k}(2, 1)) < abs(Window_normal_cam{k}(3, 1))
        v = [0, 1, 0];
    else
        v = [0, 0, 1];
    end
    u = cross(Window_normal_cam{k}(:, 1), v);
    u = u / norm(u);
    v = cross(Window_normal_cam{k}(:, 1), u);
    theta = linspace(0, 2*pi, sample_num);
    Window_circle_3D = R(2) * (u' * cos(theta) + v' * sin(theta));
    % plot3(Window_center_cam{1}(1, 1) + Window_circle_3D(1, :), Window_center_cam{1}(2, 1) + Window_circle_3D(2, :), Window_center_cam{1}(3, 1) + Window_circle_3D(3, :), 'b', 'LineWidth', 4);hold on;
    InitialMap_PM_all{k} = [PM_center_cam{k}(1) + PM_circle_3D(1, :); ...
                            PM_center_cam{k}(2) + PM_circle_3D(2, :); ...
                            PM_center_cam{k}(3) + PM_circle_3D(3, :)];

    InitialMap_Win_all{k} = [Window_center_cam{k}(1) + Window_circle_3D(1, :); ...
                             Window_center_cam{k}(2) + Window_circle_3D(2, :); ...
                             Window_center_cam{k}(3) + Window_circle_3D(3, :)];
    % axis equal;
    % grid on;
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % ylim([-10 10]);
    % zlim([-10 10]);
    % xlim([-10 10]);
    % title('3D Visualization of Cone in Camera Frame');
    % hold off;
    % f = FigureRotator();
    z_diff = (Window_center_cam{k}(3,1) + Window_circle_3D(3,:)) - ...
        (PM_center_cam{k}(3,1) + PM_circle_3D(3,:));
    % z_diff = (Window_center_cam{k}(3,1)) - (PM_center_cam{k}(3,1));

    z_means(k) = mean(z_diff);
    % z_means=z_diff;

end
[~, best_k] = min(abs(z_means));
disp(PM_center_cam{best_k});
disp(z_means);
disp(best_k);
disp(min(abs(z_means)));
InitialMap_PM = InitialMap_PM_all{best_k};
InitialMap_Win = InitialMap_Win_all{best_k};




figure;
k=best_k;
plot3(InitialMap_PM_all{k}(1, :), InitialMap_PM_all{k}(2, :), InitialMap_PM_all{k}(3, :), 'bo', 'MarkerSize', 4); hold on;
plot3(InitialMap_Win_all{k}(1, :), InitialMap_Win_all{k}(2, :), InitialMap_Win_all{k}(3, :), 'ro', 'MarkerSize', 4);

xlabel('X'); ylabel('Y'); zlabel('Z');

axis equal;
grid on;
view(3); % 3D 시점



%% Change to global frame 
for i=1:tracking_num
    T_p2c_cam{i} = T_cam_p2c{best_k} * inv(T_cam_p2c{i});%PM기준
    T_w2c_cam{i} = T_cam_w2c{best_k} * inv(T_cam_w2c{i});

end
T_p2c_iss = T_p2c_cam{best_k} * inv(T_iss_cam_true{best_k});
T_w2c_iss = T_w2c_cam{best_k} * inv(T_iss_cam_true{best_k});

for i = 1:tracking_num
    T_p2c_cam_true{i} = T_p2c_iss * T_iss_cam_true{i};
    T_w2c_cam_true{i} = T_w2c_iss * T_iss_cam_true{i};
end


%% Tracking Optimization
disp("optimization");
ransac2 = ones(1, tracking_num);
EstimatedPose{best_k} = T_p2c_cam{best_k};
EstimatedPose{best_k} = T_w2c_cam{best_k};
T_ref=EstimatedPose{best_k};
PM_normal_i = EstimatedPose{best_k} * [PM_normal_cam{best_k}; 1];
PM_normal_init = PM_normal_i(1:3)/norm(PM_normal_i(1:3));

pose_params_init = []; 	% BA 최적화에 들어갈 초기 pose 파라미터들 (logSE3 형식, 6D) // Size : 6×(tracking_num-1)
pose_index_map = [];    % 그 파라미터가 몇 번째 frame의 pose인지 기록하는 매핑 정보 //


C_pm_all = cell(1, tracking_num);
C_win_all = cell(1, tracking_num);

for i = 1:tracking_num
    C_pm_all{i} = C_pm{i};
    C_win_all{i} = C_win{i};
end

for i =1:tracking_num
    if i==best_k
        continue
    end


    T_relative = inv( inv(T_w2c_cam{best_k}) * T_w2c_cam{i} ); %  frame i의 pose를 best_k 기준으로 상대 pose로 변환
    xi = logSE3(T_relative); % SE(3) 변환 행렬을 6D vector로 바꿈 (Lie algebra)
    pose_params_init = [pose_params_init; xi]; % 위에서 계산한 6D pose 파라미터를 최적화 초기값에 쌓음
    pose_index_map(end+1) = i; % 현재 이 xi가 몇 번째 frame의 pose인지 기록 (예: 2번, 4번, ...)

end

% %% DEBUG
% % 최적화된 pose 복원
% EstimatedPose=cell(1,tracking_num);
% EstimatedPose{best_k} = T_p2c_cam{best_k};
% EstimatedPose{best_k} = T_w2c_cam{best_k};
% 
% for cnt = 1:length(pose_index_map) % 최화적화 대상이었던 frame들만 loop을 돔 (예: best_k 제외한 나머지)
%     i = pose_index_map(cnt);
%     xi = pose_params_init((cnt-1)*6+1 : cnt*6); % 최적화된 6D pose vector (logSE3)를 추출
%     T_rel = expSE3(xi); % 6D vector를 다시 SE(3) 행렬로 복원
%     T{i}=T_rel;
%     T{5}=EstimatedPose{best_k};
%     EstimatedPose{i} = EstimatedPose{best_k} * inv(T_rel);
% end
% for i = 1:tracking_num
%     plot_camera_frame(T{i}(1:3, 1:3), T{i}(1:3, 4), 0.5, 'm'); hold on;
%     plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;
%     plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 1, 'g'); hold on;
%     plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
% 
% 
%     view(-10, -45);
% 
%     grid on; axis equal;
%     xlabel('X'); ylabel('Y'); zlabel('Z');
% 
%     title("Tracking Optimization");
% 
% 
% end
%%

objective_function=@(params) BA_motiononly_cost( ...
    params, InitialMap_PM, InitialMap_Win, C_pm_all,C_win_all,intrinsics.K,best_k,pose_index_map,T_ref,ellipses_params,tstamp,imgList,imagePath,T_p2c_cam_true);

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
    'Display','iter','FiniteDifferenceType','forward');

[opt_params, resnorm] = lsqnonlin(objective_function, pose_params_init, [], [], options);

% %%
% J = computeJacobian(@(x) BA_motiononly_cost( ...
%     x, InitialMap_PM, InitialMap_Win, C_pm_all, C_win_all, intrinsics.K, best_k, ...
%     pose_index_map, T_ref, ellipses_params, tstamp, imgList, imagePath, T_p2c_cam_true), opt_params);
% H = J' * J;
% eigvals = sort(eig(H), 'descend');
% 
% % 수치적 안정성 평가
% tol = 1e-6;  % 또는 eps * norm(H)
% rank_H = sum(eigvals > tol);
% min_eig = min(eigvals);
% max_eig = max(eigvals);
% cond_H = max_eig / max(min_eig, 1e-12);  % 0 나눔 방지용
% num_small = sum(eigvals < 1e-6);
% 
% fprintf('최소 고유값: %.3e\n', min_eig);
% fprintf('최대 고유값: %.3e\n', max_eig);
% fprintf('Condition Number (H): %.3e\n', cond_H);
% fprintf('Near-zero eigenvalues (<1e-6): %d 개\n', num_small);
% 
% % 시각화
% figure;
% semilogy(eigvals, 'o-');
% xlabel('Index'); ylabel('Eigenvalue (log scale)');
% title('Eigenvalue Spectrum of Hessian');
% grid on;
% 
% % 고유값 분해 (고유벡터까지 포함)
% [V, D] = eig(H);
% [eigvals_sorted, idx] = sort(diag(D), 'ascend');   % 고유값 작은 순서대로 정렬
% V_sorted = V(:, idx);                              % 그에 맞는 고유벡터 정렬
% 
% % 가장 작은 고유값에 대응하는 고유벡터
% v_null = V_sorted(:, 1);
% 
% % 시각화: 어떤 파라미터에 영향을 많이 주는지 보기
% figure;
% bar(abs(v_null));
% xlabel('Parameter Index'); ylabel('|Component|');
% title('Component Magnitude of Smallest Eigenvector');
% grid on;
% 
% % 상위 10개 성분 분석
% [~, sort_idx] = sort(abs(v_null), 'descend');
% top_k = 100;
% names = {'ωx', 'ωy', 'ωz', 'tx', 'ty', 'tz'};
% 
% fprintf('\n🔍 Top %d influential parameters in v_null:\n', top_k);
% for i = 1:top_k
%     param_idx = sort_idx(i);
%     frame_id = floor((param_idx - 1) / 6) + 1;
%     local_idx = mod(param_idx - 1, 6) + 1;
%     fprintf('Param %3d → Frame %2d, %s (%.3e)\n', ...
%         param_idx, frame_id, names{local_idx}, v_null(param_idx));
% end

%%
% 최적화된 pose 복원
EstimatedPose=cell(1,tracking_num);
EstimatedPose{best_k} = T_p2c_cam{best_k};
EstimatedPose{best_k} = T_w2c_cam{best_k};

for cnt = 1:length(pose_index_map) % 적화 대상이었던 frame들만 loop을 돔 (예: best_k 제외한 나머지)
    i = pose_index_map(cnt);
    xi = opt_params((cnt-1)*6+1 : cnt*6); % 최적화된 6D pose vector (logSE3)를 추출
    T_rel = expSE3(xi); % 6D vector를 다시 SE(3) 행렬로 복원
    EstimatedPose{i} = EstimatedPose{best_k} * inv(T_rel);
end



for i = 1:tracking_num
    if isempty(EstimatedPose{i})
        continue
    end

    PM_normal = EstimatedPose{i} * [PM_normal_cam{i}; 1];
    PM_normal_esti = PM_normal(1:3) / norm(PM_normal(1:3));
    angle_diff2{i} = acos(dot(PM_normal_init, PM_normal_esti)) * 180 / pi;

    if angle_diff2{i} > 300
        ransac2(i) = 0;
    end
end


ransac2 = logical(ransac2);
tstamp = tstamp(ransac2);
imgList = imgList(ransac2, :);
R_iss_cam_true = R_iss_cam_true(ransac2);
t_iss_cam_true = t_iss_cam_true(ransac2);
T_iss_cam_true = T_iss_cam_true(ransac2);
T_p2c_cam = T_p2c_cam(ransac2);
T_w2c_cam = T_w2c_cam(ransac2);
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



%% Visualize Optimization Result
figure;

plot3(InitialMap_PM(1, :), InitialMap_PM(2, :), InitialMap_PM(3, :), 'c', 'MarkerSize', 10); hold on;
plot3(InitialMap_Win(1, :), InitialMap_Win(2, :), InitialMap_Win(3, :), 'm', 'MarkerSize', 10);

for i = 1:tracking_num
    % plot3(InitialMap_PM_all{i}(1, :), InitialMap_PM_all{i}(2, :), InitialMap_PM_all{i}(3, :), 'bo', 'MarkerSize', 2); hold on;
    % plot3(InitialMap_Win_all{i}(1, :), InitialMap_Win_all{i}(2, :), InitialMap_Win_all{i}(3, :), 'ro', 'MarkerSize', 2);
    if i == best_k
        continue;
    end

    plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on;
    plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;

    plot_camera_frame(EstimatedPose{best_k}(1:3, 1:3), EstimatedPose{best_k}(1:3, 4), 0.5, 'r');
    plot_camera_frame(T_p2c_cam_true{best_k}(1:3, 1:3), T_p2c_cam_true{best_k}(1:3, 4), 0.5, 'r');

    view(-10, -45);

    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');

    title("Tracking Optimization");

    % if i == 1
    %     plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    %     plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
    %     % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    %     view(-10, -45);
    %     % view(0, 0);
    %     grid on; axis equal;
    %     xlabel('X'); ylabel('Y'); zlabel('Z');
    %     title("Tracking Optimization");
    %     % pause(4);
    % end
    % plot_camera_frame(EstimatedPose{i}(1:3, 1:3), EstimatedPose{i}(1:3, 4), 0.5, 'r'); hold on; %estimated with optimization
    % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); % true cam pose
    % plot_camera_frame(T_p2c_cam{i}(1:3, 1:3), T_p2c_cam{i}(1:3, 4), 0.5, 'b');
    view(-10, -45);
    % view(0, 0);
    % plot_camera_frame(BAEstimatedPose{i}(1:3, 1:3), BAEstimatedPose{i}(1:3, 4), 0.5, 'm');
    grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("Tracking Optimization");
    % pause(1);
end
set(gcf, 'Color', 'w');

grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
% ylim([-0.2 0.8]);
% zlim([0 2.1]);
% xlim([-0.5 0.5]);
title("Tracking Optimization");
f=FigureRotator();
k=10;
hold off;


% %% Reprojection Visualization
% 
% for i = 1:tracking_num
%     % if i == best_k
%     %     continue;
%     % end
%     % tstampStr = sprintf('%.0f', tstamp(i));
%     % image = strcat(imagePath, imgList(i, :));
%     % for j = 1:sample_num
%     %     P_camera = R_relative_opt{i}* InitialMap_PM(:, j) + T_relative_opt{i};
%     %     % Convert to homogeneous coordinates by appending a row of ones
%     %     P_camera_homogeneous = P_camera;
%     % 
%     %     % Project 3D points to 2D using intrinsic matrix
%     %     projected_points = intrinsics.K * P_camera_homogeneous;
%     % 
%     %     % Convert from homogeneous to 2D coordinates
%     %     projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
%     %     projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
%     % 
%     %     % Extract the 2D points
%     %     u1(j) = projected_points(1, :);
%     %     v1(j) = projected_points(2, :);
%     % 
%     % end
%     % 
%     % for j = 1:sample_num
%     %     P_camera = R_relative_opt{i} * InitialMap_Win(:, j) + T_relative_opt{i};
%     % 
%     %     % Convert to homogeneous coordinates by appending a row of ones
%     %     P_camera_homogeneous = P_camera;
%     % 
%     %     % Project 3D points to 2D using intrinsic matrix
%     %     projected_points = intrinsics.K * P_camera_homogeneous;
%     % 
%     %     % Convert from homogeneous to 2D coordinates
%     %     projected_points(1, :) = projected_points(1, :) ./ projected_points(3, :);
%     %     projected_points(2, :) = projected_points(2, :) ./ projected_points(3, :);
%     % 
%     %     % Extract the 2D points
%     %     u2(j) = projected_points(1, :);
%     %     v2(j) = projected_points(2, :);
%     % 
%     % 
%     % end
%     xlim([0, 1280]);
%     ylim([0, 880]);
%     img = imread(image);
%     [img_height, img_width, ~] = size(img);
%     imshow(img);hold on;
%     figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
%     % plot(u1, v1, 'c', 'MarkerSize',3, 'LineWidth', 2);% Plot reprojected points in red
%     % plot(u2, v2, 'm', 'MarkerSize',3, 'LineWidth', 2); % Plot reprojected points in red
%     % title('Reprojection Result');
%     text(15, 15, sprintf('#%s ', tstampStr), 'FontSize', 15, 'color', 'r'); hold on; grid on; axis equal
% 
% 
% 
%     xlabel('X-axis');
%     ylabel('Y-axis');
%     pause(0.5);
% end 

%% Evaluation


[RPE_RMSE_MO,RPE_MO] = calcRPE(EstimatedPose, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_MO,ATE_MO] = calcATE(EstimatedPose, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_MO,RMD_MO] = calcRMD(EstimatedPose, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_p2c,RPE_p2c] = calcRPE(T_p2c_cam, T_p2c_cam_true, 5, 'RMSE');
[ATE_RMSE_p2c,ATE_p2c] = calcATE(T_p2c_cam, T_p2c_cam_true, 'RMSE');
[RMD_RMSE_p2c,RMD_p2c] = calcRMD(T_p2c_cam, T_p2c_cam_true, 'RMSE');

[RPE_RMSE_p2cw,RPE_p2cw] = calcRPE(T_w2c_cam, T_w2c_cam_true, 5, 'RMSE');
[ATE_RMSE_p2cw,ATE_p2cw] = calcATE(T_w2c_cam, T_w2c_cam_true, 'RMSE');
[RMD_RMSE_p2cw,RMD_p2cw] = calcRMD(T_w2c_cam, T_w2c_cam_true, 'RMSE');
% 
% [RPE_RMSE_p2cw,RPE_p2cw] = calcRPE(T_p2c_cam_true, T_w2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_p2cw,ATE_p2cw] = calcATE(T_p2c_cam_true, T_w2c_cam_true, 'RMSE');
% [RMD_RMSE_p2cw,RMD_p2cw] = calcRMD(T_p2c_cam_true, T_w2c_cam_true, 'RMSE');

% [RPE_RMSE_droid,RPE_droid] = calcRPE(T_p2c_droidcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_droid,ATE_droid] = calcATE(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_droid,RMD_droid] = calcRMD(T_p2c_droidcam, T_p2c_cam_true, 'RMSE');
% 
% [RPE_RMSE_colmap,RPE_colmap] = calcRPE(T_p2c_colmapcam, T_p2c_cam_true, 5, 'RMSE');
% [ATE_RMSE_colmap,ATE_colmap] = calcATE(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');
% [RMD_RMSE_colmap,RMD_colmap] = calcRMD(T_p2c_colmapcam, T_p2c_cam_true, 'RMSE');

% reprojection_error_total = ( rms(e1) + rms(e2) ) / 2;
% reprojection_error1 = rms(e1);
% reprojection_error2 = rms(e2);

% fprintf("BA result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_BA, RPE_RMSE_BA, RMD_RMSE_BA);
fprintf("Motion Opt result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_MO, RPE_RMSE_MO, RMD_RMSE_MO);
fprintf("p2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2c, RPE_RMSE_p2c, RMD_RMSE_p2c);
% fprintf("DROID result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_droid, RPE_RMSE_droid, RMD_RMSE_droid);
% fprintf("COLMAP result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_colmap, RPE_RMSE_colmap, RMD_RMSE_colmap);
% fprintf("reprojection error result \n PM: %f \n Win %f \n total %f \n", rms(e1), rms(e2), reprojection_error_total);

% fprintf("Motion Opt result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_MOw, RPE_RMSE_MOw, RMD_RMSE_MOw);
fprintf("w2c result \n ATE: %f \n RPE %f \n RMD %f \n", ATE_RMSE_p2cw, RPE_RMSE_p2cw, RMD_RMSE_p2cw);


%%

function residuals = BA_motiononly_cost(params, PM3D, WIN3D, C_pm_all, C_win_all, K, best_k, pose_index_map,T_ref,ellipses_params,tstamp,imgList,imagePath,T_p2c_cam_true)
    N = length(pose_index_map);
    EstimatedPose = cell(1, N+1);
    % figure;
    

    % Reconstruct all poses from parameters
    for cnt = 1:N
        xi = params((cnt-1)*6+1 : cnt*6);
        T_rel = expSE3(xi);
        idx = pose_index_map(cnt);
        Pose{idx} = T_rel;
        Pose{best_k}=T_ref;
    end

    % Compute residuals
    residuals = [];

    for i = 1:length(Pose)
        if isempty(Pose{i})
            continue
        end
        image = strcat(imagePath, imgList(i, :));

        Qpm = C_pm_all{i};
        Qwin = C_win_all{i};
        
        % Project PM and WIN 3D to 2D
        P1 = K * (Pose{i}(1:3, :) * [PM3D; ones(1, size(PM3D, 2))]);
        P2 = K * (Pose{i}(1:3, :) * [WIN3D; ones(1, size(WIN3D, 2))]);

        x1 = P1(1:2, :) ./ P1(3, :);
        x2 = P2(1:2, :) ./ P2(3, :);

        % homogeneous coordinates
        x1_h = [x1; ones(1, size(x1, 2))];
        x2_h = [x2; ones(1, size(x2, 2))];

        % %DEBUG 시각화
        % plot_camera_frame(Pose{i}(1:3, 1:3), Pose{i}(1:3, 4), 0.5, 'r'); hold on;
        % view(-10, -45);
        % grid on; axis equal;
        % xlabel('X'); ylabel('Y'); zlabel('Z');
        % title("Tracking Optimization");
        % figure;
        % xlim([0, 1280]);
        % ylim([0, 880]);
        % img = imread(image);
        % tstampStr = sprintf('%.0f', tstamp(i));
        % imshow(img);hold on;
        % figure_detected_two_ellipse(image, ellipses_params{i}, tstampStr);
        % xx1 = P1(1, :) ./ P1(3, :);  
        % yy1 = P1(2, :) ./ P1(3, :);  
        % plot(xx1, yy1, 'c.', 'MarkerSize', 3);  
        % xx2 = P2(1, :) ./ P2(3, :);  
        % yy2 = P2(2, :) ./ P2(3, :);  
        % plot(xx2, yy2, 'm.', 'MarkerSize', 3);  
        % title('Reprojection Result');
        % text(15, 15, sprintf('#%s ', tstampStr), 'FontSize', 15, 'color', 'r'); hold on; grid on; axis equal
        % pause(0.5);

        % 
        % plot_camera_frame(Pose{i}(1:3, 1:3), Pose{i}(1:3, 4), 0.5, 'r'); hold on;
        % plot_camera_frame(T_p2c_cam_true{i}(1:3, 1:3), T_p2c_cam_true{i}(1:3, 4), 0.5, 'k'); hold on;
        % view(-10, -45);
        % grid on; axis equal;
        % xlabel('X'); ylabel('Y'); zlabel('Z');
        % title("Tracking Optimization");

    
        for j = 1:size(x1_h, 2)
            r1 = x1_h(:, j)' * Qpm * x1_h(:, j);
            r2 = x2_h(:, j)' * Qwin * x2_h(:, j);
            residuals = [residuals; r1];
        end
    end
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

function draw_conic_Q_on_image(Q, color, imageSize)
    [H, W] = deal(imageSize(1), imageSize(2));
    [U, V] = meshgrid(1:W, 1:H);

    % Construct homogeneous coordinates for every pixel
    X = [U(:)'; V(:)'; ones(1, numel(U))];
    
    % Evaluate xᵀQx for all pixels
    vals = sum(X .* (Q * X), 1);
    vals_img = reshape(vals, H, W);

    % Draw contour where xᵀQx ≈ 0 (i.e., the ellipse)
    contour(U, V, vals_img, [0 0], color, 'LineWidth', 2);
end



function J = computeJacobian(fun, x0)
    eps = 1e-6;
    fx = fun(x0);
    n = length(x0);
    m = length(fx);
    J = zeros(m, n);
    for i = 1:n
        dx = zeros(size(x0));
        dx(i) = eps;
        J(:, i) = (fun(x0 + dx) - fx) / eps;
    end
end
