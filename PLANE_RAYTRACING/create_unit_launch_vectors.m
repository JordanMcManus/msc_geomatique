function launch_vectors = create_unit_launch_vectors(azimuth_time_series, beam_time_series)
    % azimuth_time_series = [time, azimuth, azimuth_non_modulo]
    % beam_time_series = [beam_time, beam_elevation, beam_id]
    % [beam_time, beam_elevation, beam_id, beam_vertical_correction]
    
    beam_time = beam_time_series(:, 1); % (n x 1)
    
    interp_azimuth_non_modulo = interp1(azimuth_time_series(:, 1), azimuth_time_series(:, 3), beam_time);
    interp_azimuth = mod(interp_azimuth_non_modulo, 360);
    
    
    beam_elevation = beam_time_series(:, 2); % (n x 1)
    beam_id = beam_time_series(:, 3); % (n x 1)
    beam_vertical_correction = beam_time_series(:, 4); % (n x 1)
    
    x = cosd(beam_elevation).*sind(interp_azimuth);
    y = cosd(beam_elevation).*cosd(interp_azimuth);
    z = sind(beam_elevation);
    
    % launch_vectors = [beam_time, x, y, z, azimuth, elevation, laser_id, beam_vertical_correction];
    launch_vectors = [beam_time, x, y, z, interp_azimuth, beam_elevation, beam_id, beam_vertical_correction];
end