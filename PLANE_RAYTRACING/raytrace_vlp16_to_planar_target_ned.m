function [data_on_target, data_on_infinite_plane] = raytrace_vlp16_to_planar_target_ned(sbet_ned, start_time, end_time, scanner_configuration, target_plane)

    %% VLP-16 configuration
    start_azimuth = scanner_configuration.start_azimuth;
    rpm = scanner_configuration.rpm;
    azimuth_time_delta = scanner_configuration.azimuth_time_delta;
    
    %% Plane parameters
    point_on_plane = target_plane.point_on_plane;
    plane_normal = target_plane.unit_normal;
    target_min_bounds = target_plane.target_min_bounds;
    target_max_bounds = target_plane.target_max_bounds;

    %% Plateform parameters
    lever_arm = scanner_configuration.lever_arm;
    boresight_degrees = scanner_configuration.boresight_degrees;
    assert(size(boresight_degrees,1) == 1, 'boresight parameters must be (1x3)');
    assert(size(boresight_degrees,2) == 3, 'boresight parameters must be (1x3)');
    boresightMatrix = get_ned_dcm_degrees(boresight_degrees);
    
    %% Azimuth time series
    [azimuth_time, azimuth_value, azimuth_non_modulo] = generate_azimuth_time_series(start_azimuth, rpm, start_time, end_time, azimuth_time_delta);
    
    %% Beam time series
    [beam_time, beam_elevation, beam_id, beam_vertical_correction] = generate_beam_time_series(start_time, end_time);
    
    %% Create launch vectors
    azimuth_time_series = [azimuth_time, azimuth_value, azimuth_non_modulo];
    beam_time_series = [beam_time, beam_elevation, beam_id, beam_vertical_correction];
    launch_vectors = create_unit_launch_vectors(azimuth_time_series, beam_time_series);
    
    %% Match SBET and unit launch vectors
    assert(launch_vectors(1,1) >= sbet_ned(1,1), 'initial beam time must be after initial SBET time');
    assert(launch_vectors(end,1) <= sbet_ned(end,1), 'initial beam time must be after initial SBET time');
    sbet_launch_vectors = match_launch_vectors_sbet(sbet_ned, launch_vectors);
    
    %% VLP-16 optical center
    roll = sbet_launch_vectors(:, 5);
    pitch = sbet_launch_vectors(:, 6);
    heading = sbet_launch_vectors(:, 7);

    lever_arm_ned = reorient_in_ned(roll, pitch, heading, repmat(lever_arm', size(sbet_launch_vectors,1), 1));
    optical_center_ned = bsxfun(@plus, sbet_launch_vectors(:, 2:4), lever_arm_ned);
    
    %% Reorient launch vectors
    launch_vectors_ins = (boresightMatrix*launch_vectors(:,2:4)')';
    launch_vectors_ned = reorient_in_ned(roll, pitch, heading, launch_vectors_ins);
    
    %% Raytracing to target plane
    denominator = launch_vectors_ned*plane_normal';
    oc_to_plane_point = bsxfun(@minus, point_on_plane, optical_center_ned);
    plane_inc_normal_ned_multiplicity = repmat(plane_normal, size(oc_to_plane_point,1), 1);
    numerator = dot(oc_to_plane_point, plane_inc_normal_ned_multiplicity, 2);
    
    distance_from_VLP_to_intersection = zeros(size(launch_vectors_ned,1), 1);
    index_non_zero_denominator = abs(denominator) > 0.001;
    distance_from_VLP_to_intersection(index_non_zero_denominator) = numerator(index_non_zero_denominator) ./ denominator(index_non_zero_denominator);
    
    %% Remove negative distances (target is facing away from launch vector)
    distance_from_VLP_to_intersection(distance_from_VLP_to_intersection < 0) = 0;
    
    %% coordinates of raytraced points
    intersection_vlp_coordinates = bsxfun(@times, launch_vectors_ned, distance_from_VLP_to_intersection);
    intersection_points = bsxfun(@plus, intersection_vlp_coordinates, optical_center_ned);
    
    %% Out of bounds: 
    index_out_X = intersection_points(:,1) < target_min_bounds(1) | intersection_points(:,1) > target_max_bounds(1);
    index_out_Y = intersection_points(:,2) < target_min_bounds(2) | intersection_points(:,2) > target_max_bounds(2);
    index_out_Z = intersection_points(:,3) < target_min_bounds(3) | intersection_points(:,3) > target_max_bounds(3);
    
    %% keep points inside target 3D bounding box
    keep_index = ~index_out_X & ~index_out_Y & ~index_out_Z;
    
    %% output format: [x_ned, y_ned, z_ned, time, prp_x, prp_y, prp_z, roll, pitch, heading, x_launch, y_launch, z_launch, azimuth, elevation, laser_id, beam_vertical_correction, range_to_target]
    data_on_target = [ intersection_points(keep_index,:) sbet_launch_vectors(keep_index,:) distance_from_VLP_to_intersection(keep_index)];
    data_on_infinite_plane = [ intersection_points sbet_launch_vectors distance_from_VLP_to_intersection];
end