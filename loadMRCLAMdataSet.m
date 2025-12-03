% UTIAS Multi-Robot Cooperative Localization and Mapping Dataset
% produced by Keith Leung (keith.leung@robotics.utias.utoronto.ca) 2009

n_robots = 5;

disp('Parsing Dataset')
disp('Reading barcode numbers')
[subject_num, barcode_num] = textread('MRCLAM_Dataset1/Barcodes.dat', '%u %u','commentstyle','shell');
data.Barcodes = [subject_num, barcode_num];
clear subject_num barcode_num;

disp('Reading landmark groundtruth')
[subject_num, x, y, x_sd, y_sd] = textread('MRCLAM_Dataset1/Landmark_Groundtruth.dat', '%f %f %f %f %f','commentstyle','shell');
data.Landmark_Groundtruth = [subject_num x y x_sd y_sd];
clear subject_num x y x_sd y_sd;

for i = 1:n_robots
    
    disp(['Reading robot ' num2str(i) ' groundtruth'])
    [time, x, y, theta] = textread(['MRCLAM_Dataset1/Robot' num2str(i) '_Groundtruth.dat'], ...
                                   '%f %f %f %f','commentstyle','shell');
    data.Robot(i).Groundtruth = [time x y theta];
    clear time x y theta;

    disp(['Reading robot ' num2str(i) ' odometry'])
    [time, v, w] = textread(['MRCLAM_Dataset1/Robot' num2str(i) '_Odometry.dat'], ...
                            '%f %f %f','commentstyle','shell');
    data.Robot(i).Odometry = [time v w];
    clear time v w;

    disp(['Reading robot ' num2str(i) ' measurements'])
    [time, barcode_num, r, b] = textread(['MRCLAM_Dataset1/Robot' num2str(i) '_Measurement.dat'], ...
                                         '%f %f %f %f','commentstyle','shell');
    data.Robot(i).Measurement = [time barcode_num r b];
    clear time barcode_num r b;

end

disp('Parsing Complete')
