function [ARKitPointsXYZ, ARKitPointsRGB] = readXYZ_colorFile(datasetPath, interval)

% read XYZ_color.txt file
textFileDir = [datasetPath 'XYZ_color.txt'];
textData = importdata(textFileDir, ',');
textData = textData(1:interval:end,:);


% split XYZ and RGB of 3D point cloud 
ARKitPointsXYZ = textData(:,1:3).';
ARKitPointsRGB = textData(:,4:6).';


end

