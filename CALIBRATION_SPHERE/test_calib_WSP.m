clear;
clc;

%% load sbet
sbet_filename = 'sbet_calib_port_quebec_WSP.out';
sbet = readSBET(sbet_filename);
disp(['load sbet file: ' sbet_filename])

%% llh = latitude longitude height
% Can use any point, for example:
% 1. first point of SBET
% 2. average of multiple SBET
% 3. known target coordinates
llh_0 = mean(sbet(:,2:4));
disp('== CRS origin ==')
disp(['Longitude: ' num2str(rad2deg(llh_0(1)), 15)]);
disp(['Latitude: ' num2str(rad2deg(llh_0(2)), 15)]);
disp(['Height: ' num2str(llh_0(3), 15)]);

%% LGF NED conversion
[sbet_ned, conversion_struct] = transform_sbet_to_ned_lgf(sbet, llh_0);

%% load SMRMSG
smrmsg_filename = 'smrmsg_calib_port_quebec_WSP.out';
[~, sbet_accuracy] = readPosPacAccuracy(smrmsg_filename);
disp(['load SMRMSG: ' smrmsg_filename])

%% generate points on spheres with known center and radius for visualisation
c1_trx = [1409031.843, -4139132.975, 4627982.702];
c1_ned = ecef_to_ned(c1_trx, conversion_struct);
sphere_points_1 = buildSphere(c1_ned.', 0.19, 20);
writeMatrixCsv('wsp_viz_sphere_points_1.txt', sphere_points_1);

c2_trx = [1409049.348, -4139124.164, 4627985.253];
c2_ned = ecef_to_ned(c2_trx, conversion_struct);
sphere_points_2 = buildSphere(c2_ned.', 0.19, 20);
writeMatrixCsv('wsp_viz_sphere_points_2.txt', sphere_points_2);

%% raw vlp16 mat file for first sphere
sphere_1_filename = 'sphere_WSP_1.mat';
load(sphere_1_filename);

%% select vlp16 mat file for second sphere
sphere_2_filename = 'sphere_WSP_2.mat';
load(sphere_2_filename);

%% vlp accuracy TODO: must calculated during cartesian conversion
% for now estimate x, y, z accuracies to 15 mm
vlp_accuracy = [0.015, 0.015 0.015];

%% sphere and plane structures
sphere_struct = {};
sphere_struct.num_spheres = 2;

%% sphere 1
sphere_struct.radius{1} = 0.19;
sphere_struct.center{1} = c1_ned;
sphere_struct.vlp_sphere{1} = sphere_WSP_1;
[sbet_lidar_1, intensity_1] = match_raw_vlp16_sbet_accuracy(sbet_ned, sbet_accuracy, sphere_struct.vlp_sphere{1}, vlp_accuracy);
sphere_struct.matched_vlp_data{1} = sbet_lidar_1;

%% sphere 2
sphere_struct.radius{2} = 0.19;
sphere_struct.center{2} = c2_ned;
sphere_struct.vlp_sphere{2} = sphere_WSP_2;
[sbet_lidar_2, intensity_2] = match_raw_vlp16_sbet_accuracy(sbet_ned, sbet_accuracy, sphere_struct.vlp_sphere{2}, vlp_accuracy);
sphere_struct.matched_vlp_data{2} = sbet_lidar_2;

%% plateform parameters
nominal_boresight_degrees = [180, 45, 0];
nominal_lever_arm = [1.363, -1.22, -1.402];
initial_X = [nominal_boresight_degrees, nominal_lever_arm];

%% matched georef
georef_ned_s1 = regeoref_ned(sbet_lidar_1, nominal_boresight_degrees, nominal_lever_arm);
georef_ned_s2 = regeoref_ned(sbet_lidar_2, nominal_boresight_degrees, nominal_lever_arm);
writeMatrixCsv('wsp_matched_georef_s1.txt', georef_ned_s1);
writeMatrixCsv('wsp_matched_georef_s2.txt', georef_ned_s2);

%% calibration
calibrated_boresight_lever_arm = boresight_lever_arm_calibration(sphere_struct, nominal_boresight_degrees, nominal_lever_arm)
calibrated_boresight_degrees = calibrated_boresight_lever_arm(1:3);
calibrated_lever_arm = calibrated_boresight_lever_arm(4:6);

%% georef after calibration
calib_georef_ned_s1 = regeoref_ned(sbet_lidar_1, calibrated_boresight_degrees, calibrated_lever_arm);
calib_georef_ned_s2 = regeoref_ned(sbet_lidar_2, calibrated_boresight_degrees, calibrated_lever_arm);
filename_sphere_1 = 'calib_georef_s1.txt';
filename_sphere_2 = 'calib_georef_s2.txt';
writeMatrixCsv(filename_sphere_1, calib_georef_ned_s1);
writeMatrixCsv(filename_sphere_2, calib_georef_ned_s2);

