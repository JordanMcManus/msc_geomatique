function [data_on_sphere, all_data] = raytrace_vlp16_to_sphere_target_ned(sbet_ned, start_time, end_time, scanner_configuration, target_sphere)

    %% VLP-16 configuration
    start_azimuth = scanner_configuration.start_azimuth;
    rpm = scanner_configuration.rpm;
    azimuth_time_delta = scanner_configuration.azimuth_time_delta;
    
    %% Plateform parameters
    lever_arm = scanner_configuration.lever_arm;
    boresight_degrees = scanner_configuration.boresight_degrees;
    assert(size(boresight_degrees,1) == 1, 'boresight parameters must be (1x3)');
    assert(size(boresight_degrees,2) == 3, 'boresight parameters must be (1x3)');
    boresightMatrix = get_ned_dcm_degrees(boresight_degrees);
    
    %% Target sphere
    sphere_radius = target_sphere.radius;
    sphere_center = target_sphere.sphere_center_ned;
    
    %% Azimuth time series
    [azimuth_time, azimuth_value, azimuth_non_modulo] = generate_azimuth_time_series(start_azimuth, rpm, start_time, end_time, azimuth_time_delta);
    
    %% Beam time series
    [beam_time, beam_elevation, beam_id, beam_vertical_correction] = generate_vlp16_beam_time_series(start_time, end_time);
    
    %% Create launch vectors
    azimuth_time_series = [azimuth_time, azimuth_value, azimuth_non_modulo];
    beam_time_series = [beam_time, beam_elevation, beam_id, beam_vertical_correction];
    launch_vectors = create_unit_launch_vectors(azimuth_time_series, beam_time_series);
    
    %% Match SBET and unit launch vectors
    assert(launch_vectors(1,1) >= sbet_ned(1,1), 'initial beam time must be after initial SBET time');
    assert(launch_vectors(end,1) <= sbet_ned(end,1), 'initial beam time must be after initial SBET time');
    sbet_launch_vectors = match_launch_vectors_sbet(sbet_ned, launch_vectors);
    N_beams = size(sbet_launch_vectors,1);
    
    %% VLP-16 optical center
    roll = sbet_launch_vectors(:, 5);
    pitch = sbet_launch_vectors(:, 6);
    heading = sbet_launch_vectors(:, 7);

    lever_arm_ned = reorient_in_ned(roll, pitch, heading, repmat(lever_arm', N_beams, 1));
    optical_center_ned = bsxfun(@plus, sbet_launch_vectors(:, 2:4), lever_arm_ned);
    
    %% Reorient launch vectors
    launch_vectors_ins = (boresightMatrix*launch_vectors(:,2:4)')';
    launch_vectors_ned = reorient_in_ned(roll, pitch, heading, launch_vectors_ins);
    
    %% line point analysis (potential for optimisation)
    omc = bsxfun(@minus, optical_center_ned, sphere_center.');
    
    c = cross(omc,launch_vectors_ned,2);
    dc = dot(c,c,2);
    
    I_test = dc < sphere_radius; % these beams hit the sphere but don't know where yet
    % to optimise, only use these points in calculations below.
    % for now, no optimisation.
    
    
    
    %% Raytracing to target sphere
    r2 = repmat(sphere_radius*sphere_radius, N_beams, 1);
    udomc = dot(launch_vectors_ned,omc,2);
    delta = udomc.*udomc - dot(omc,omc,2) + r2;
    
    I0 = delta < 0;
    I2 = delta > 0;
    I1 = ~I0 & ~I2; % considering sphere skin of zero thickness
    % might be interesting to choose a non-zero skin thickness to analyse
    % beam tangency
    
    distance_from_VLP_to_intersection = zeros(N_beams, 1);
    distance_from_VLP_to_intersection(I1) = -udomc(I1);
    distance_from_VLP_to_intersection(I2) = -udomc(I2) - sqrt(delta(I2));
    distance_from_VLP_to_intersection(distance_from_VLP_to_intersection < 0) = 0; % negative distance means beam is traveling away from sphere
    
    I_sphere = distance_from_VLP_to_intersection > 0;
    
    
    intersection_vlp_coordinates = bsxfun(@times, launch_vectors_ned, distance_from_VLP_to_intersection);
    intersection_points = bsxfun(@plus, intersection_vlp_coordinates, optical_center_ned);
    
    all_data = [intersection_points sbet_launch_vectors distance_from_VLP_to_intersection];
    data_on_sphere = [intersection_points(I_sphere, :) sbet_launch_vectors(I_sphere, :) distance_from_VLP_to_intersection(I_sphere)];
    % data_on_infinite_plane = [ intersection_points sbet_launch_vectors distance_from_VLP_to_intersection];
end