clear;
clc;

%% VLP-16 configuration
firing_cycle_duration = 55.296e-6;
scanner_configuration.start_azimuth = 180;
scanner_configuration.rpm = 300;
scanner_configuration.azimuth_time_delta = 2*firing_cycle_duration;
disp('VLP-16 configuration')
disp(['scanner_configuration.start_azimuth: ' num2str(scanner_configuration.start_azimuth)])
disp(['scanner_configuration.azimuth_time_delta: ' num2str(scanner_configuration.azimuth_time_delta)])
disp(['scanner_configuration.rpm: ' num2str(scanner_configuration.rpm)])


%% Plateform parameters
scanner_configuration.nominal_lever_arm = [1.42; -1.36; -1.43];
scanner_configuration.nominal_boresight_degrees = [180, -45, 0];
scanner_configuration.lever_arm = [1.5; -1.24; -1.36];
scanner_configuration.boresight_degrees = [179.5, -44.9, 1.2];
disp('Plateform parameters')
disp('scanner_configuration.lever_arm:')
scanner_configuration.lever_arm
disp('scanner_configuration.boresight_degrees:')
scanner_configuration.boresight_degrees

%% Timestamps for different passages
start_time = [144625, 144912, 145212, 145254, 145359, 145480, 143553, 143682, 143810, 144368, 144733, 145040, 145324, 144497, 144824, 145132, 145405];
end_time = [144645, 144932, 145232, 145274, 145379, 145500, 143573, 143702, 143830, 144388, 144753, 145060, 145344, 144517, 144844, 145152, 145425];


%% Sphere target configuration: Lat, Lon, Alt (data in gps format)
% config sphere 1 48°28'50.89"N    68°31'3.34"W
sphere1_alt = -19.756; % 5.828 m orthometric
sphere1_lat_deg = 48;
sphere1_lat_min = 28;
sphere1_lat_sec = 50.89;
sphere1_lat = sphere1_lat_deg + sphere1_lat_min/60 + sphere1_lat_sec/3600;

sphere1_lon_deg = 68;
sphere1_lon_min = 31;
sphere1_lon_sec = 3.34;
sphere1_lon = -(sphere1_lon_deg + sphere1_lon_min/60 + sphere1_lon_sec/3600);

s1_llh = [deg2rad(sphere1_lat), deg2rad(sphere1_lon), sphere1_alt];

% config sphere 2  48°28'50.64"N     68°31'2.95"W
sphere2_alt = -19.486; % 6.098 m orthometric
sphere2_lat_deg = 48;
sphere2_lat_min = 28;
sphere2_lat_sec = 50.64;
sphere2_lat = sphere2_lat_deg + sphere2_lat_min/60 + sphere2_lat_sec/3600;

sphere2_lon_deg = 68;
sphere2_lon_min = 31;
sphere2_lon_sec = 2.95;
sphere2_lon = -(sphere2_lon_deg + sphere2_lon_min/60 + sphere2_lon_sec/3600);

s2_llh = [deg2rad(sphere2_lat), deg2rad(sphere2_lon), sphere2_alt];

% config sphere 3  48°28'51.55"N     68°31'3.47"W
sphere3_alt = -19.691; % 5.894 m orthometric
sphere3_lat_deg = 48;
sphere3_lat_min = 28;
sphere3_lat_sec = 51.55;
sphere3_lat = sphere3_lat_deg + sphere3_lat_min/60 + sphere3_lat_sec/3600;

sphere3_lon_deg = 68;
sphere3_lon_min = 31;
sphere3_lon_sec = 3.47;
sphere3_lon = -(sphere3_lon_deg + sphere3_lon_min/60 + sphere3_lon_sec/3600);

s3_llh = [deg2rad(sphere3_lat), deg2rad(sphere3_lon), sphere3_alt];

%% read SBET
sbet_filename = 'sbet_Mission 1.out';
sbet = readSBET(sbet_filename);

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

%% convert sphere centers from geographic to NED
centers_geo = [s1_llh; s2_llh; s3_llh];
centers_ned = transform_geo_to_ned(centers_geo, conversion_struct);


%% sphere 1 ray tracing
target_sphere1.name = 'Sphere 1';
target_sphere1.filename_prefix = 'sphere_1';
target_sphere1.sphere_center_ned = centers_ned(1, :).';
target_sphere1.radius = 0.19;

%% Example Raytracing
disp(['Start time: ' num2str(start_time(1)) ' End time: ' num2str(end_time(1))]);
   
file_timestamp = [num2str(start_time(1)) '_' num2str(end_time(1))];
filename_sphere_1_target = [target_sphere1.filename_prefix, '_', file_timestamp];
filename_sphere_1_target_mat = [filename_sphere_1_target, '.mat'];
filename_sphere_1_target_txt = [filename_sphere_1_target, '.txt'];
disp(filename_sphere_1_target);

[data_on_sphere_1, ~] = raytrace_vlp16_to_sphere_target_ned(sbet_ned, start_time(1), end_time(1), scanner_configuration, target_sphere1);

save(filename_sphere_1_target, 'data_on_sphere_1');
writeMatrixCsv(filename_sphere_1_target_txt, data_on_sphere_1); % can be viewed in Cloud Compare
disp(filename_sphere_1_target_txt);