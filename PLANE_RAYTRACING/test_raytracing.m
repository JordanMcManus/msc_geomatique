clear;
clc;

%% VLP-16 configuration
firing_cycle_duration = 55.296e-6;
scanner_configuration.start_azimuth = 180;

rpm = 300
scanner_configuration.rpm = rpm;
scanner_configuration.azimuth_time_delta = 2*firing_cycle_duration;
    
%% inclined Plane parameters
plane_inc_ned = [0.564284, 0.548772, -0.616791, 84.5198];
plane_inc_ned_general_form = get_plane_general_form(plane_inc_ned);
inclined_target_plane.point_on_plane = get_point_on_plane(plane_inc_ned_general_form);
inclined_target_plane.unit_normal = plane_inc_ned_general_form(1:3);
inclined_target_plane.target_min_bounds = [86.1336, -249.9359, -5.7969];
inclined_target_plane.target_max_bounds = [87.2742, -248.7974, -5.2231];

%% vertical Plane parameters
plane_vert_ned = [-0.717445, -0.696455, -0.014969, -111.37054];
plane_vert_ned_general_form = get_plane_general_form(plane_vert_ned);
vertical_target_plane.point_on_plane = get_point_on_plane(plane_vert_ned_general_form);
vertical_target_plane.unit_normal = plane_vert_ned_general_form(1:3);
vertical_target_plane.target_min_bounds = [86.3938, -249.7895, -4.8599];
vertical_target_plane.target_max_bounds = [87.3170, -248.8188, -3.5766];

%% Plateform parameters
scanner_configuration.lever_arm = [1.5; -1.24; -1.36];
scanner_configuration.boresight_degrees = [179.5 -44.9 1.2]; % [180 -45 0]

%% SBET
sbet_filename = 'sbet_Mission 1.out';
sbet = readSBET(sbet_filename);

%% Convert trajectory to NED
[sbet_ned, conversion_struct] = transform_sbet_to_ned_lgf(sbet);


%% Define Time interval for this test
start_time = 145040;
end_time = 145060;

%% raytracing
tic
[data_on_inclined_target, data_on_inclined_infinite_plane] = raytrace_vlp16_to_planar_target_ned(sbet_ned, start_time, end_time, scanner_configuration, inclined_target_plane);
toc

tic
[data_on_vertical_target, data_on_vertical_infinite_plane] = raytrace_vlp16_to_planar_target_ned(sbet_ned, start_time, end_time, scanner_configuration, vertical_target_plane);
toc

%% Gather data for one beam
data_one_raytraced_beam = data_on_inclined_target(1,:);

intersection_point = data_one_raytraced_beam(1:3).'
timestamp = data_one_raytraced_beam(4)
prp_vector = data_one_raytraced_beam(5:7).'
rph = data_one_raytraced_beam(8:10)
unit_launch_vector = data_one_raytraced_beam(11:13).'

range = data_one_raytraced_beam(18)
vertical_correction = data_one_raytraced_beam(17)
beam_vector = range * unit_launch_vector
beam_vector_corrected = beam_vector + [0;0;vertical_correction]


%% test that georef corresponds to raytraced intersection
orientation_matrix = get_ned_dcm_radians(rph);
boresight_matrix = get_ned_dcm_degrees(scanner_configuration.boresight_degrees);
lever_arm_vector = scanner_configuration.lever_arm;

georef_vector = georef_time_independant_and_frame_independant(beam_vector, prp_vector, orientation_matrix, boresight_matrix, lever_arm_vector)
intersection_point


