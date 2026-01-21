function data = loadMRCLAMdataSet(datasetDir)
%LOADMRCLAMDATASET Load MRCLAM raw dataset from folder.
%   data = loadMRCLAMdataSet(datasetDir)
%
% datasetDir example: 'data/raw/MRCLAM_Dataset1' (folder containing Barcodes.dat, etc.)

    if nargin < 1 || isempty(datasetDir)
        datasetDir = 'MRCLAM_Dataset1'; % default (relative)
    end

    if ~isfolder(datasetDir)
        error('Dataset folder not found: %s', datasetDir);
    end

    n_robots = 5;

    disp('Parsing Dataset')
    disp('Reading barcode numbers')
    [subject_num, barcode_num] = textread(fullfile(datasetDir,'Barcodes.dat'), ...
        '%u %u','commentstyle','shell');
    data.Barcodes = [subject_num, barcode_num];
    clear subject_num barcode_num;

    disp('Reading landmark groundtruth')
    [subject_num, x, y, x_sd, y_sd] = textread(fullfile(datasetDir,'Landmark_Groundtruth.dat'), ...
        '%f %f %f %f %f','commentstyle','shell');
    data.Landmark_Groundtruth = [subject_num x y x_sd y_sd];
    clear subject_num x y x_sd y_sd;

    for i = 1:n_robots

        disp(['Reading robot ' num2str(i) ' groundtruth'])
        [time, x, y, theta] = textread(fullfile(datasetDir, sprintf('Robot%d_Groundtruth.dat', i)), ...
            '%f %f %f %f','commentstyle','shell');
        data.Robot(i).Groundtruth = [time x y theta];
        clear time x y theta;

        disp(['Reading robot ' num2str(i) ' odometry'])
        [time, v, w] = textread(fullfile(datasetDir, sprintf('Robot%d_Odometry.dat', i)), ...
            '%f %f %f','commentstyle','shell');
        data.Robot(i).Odometry = [time v w];
        clear time v w;

        disp(['Reading robot ' num2str(i) ' measurements'])
        [time, barcode_num, r, b] = textread(fullfile(datasetDir, sprintf('Robot%d_Measurement.dat', i)), ...
            '%f %f %f %f','commentstyle','shell');
        data.Robot(i).Measurement = [time barcode_num r b];
        clear time barcode_num r b;

    end

    disp('Parsing Complete')
end
