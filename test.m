% Define paths
image_folder = './data/ff_return_journey_forward/gray_selected/'; % Change this to your image folder path
groundtruth_file = './data/ff_return_journey_forward/groundtruth.txt';
output_file = './data/ff_return_journey_forward/groundtruth_selected.txt';

% Step 1: Read timestamps from image filenames
image_files = dir(fullfile(image_folder, '*.png'));
timestamps = arrayfun(@(x) erase(x.name, '.png'), image_files, 'UniformOutput', false);

% Step 2: Read ground truth data
groundtruth_data = readmatrix(groundtruth_file, 'Delimiter', ' ');

% Step 3: Extract matching rows
selected_rows = [];
for i = 1:length(timestamps)
    tstamp = str2double(timestamps{i});
    
    % Find matching row in groundtruth_data
    row_idx = find(groundtruth_data(:, 1) == tstamp);
    
    if ~isempty(row_idx)
        selected_rows = [selected_rows; groundtruth_data(row_idx, :)];
    end
end

% Step 4: Save the selected rows to a new file
writematrix(selected_rows, output_file, 'Delimiter', ' ');

disp(['Filtered ground truth saved to: ', output_file]);
