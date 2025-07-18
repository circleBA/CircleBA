imagePath = 'data/iva_kibo_rot/gray/';
ellipsePath = 'data/iva_kibo_rot/results_p2c/';
posePath = 'data/iva_kibo_rot/groundtruth.txt';



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
