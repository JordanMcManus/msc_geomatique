function [azimuth_time, azimuth_value, azimuth_non_modulo] = generate_azimuth_time_series(start_azimuth, rpm, start_time, end_time, time_delta)
    assert(end_time > start_time, 'end_time must be greater than start_time');
    
    % Since we want to interpolate, make sure this time series has data
    % before and after
    azimuth_time_start = start_time - 4*time_delta;
    azimuth_time_end = end_time + 4*time_delta;
    
    duration = azimuth_time_end - azimuth_time_start;
    
    n_azimuth = floor(duration/time_delta);
    
    azimuth_resolution = rpm*(1/60)*(360)*time_delta;
    
    azimuth_number = (0:1:n_azimuth)';
    azimuth_time = azimuth_number * time_delta + azimuth_time_start;
    
    % start_time - azimuth_time(1)
    assert(azimuth_time(1) - start_time < 0, 'azimuth start_time must be smaller then specified start_time');
    assert(azimuth_time(end) - end_time > 0, 'azimuth end_time must be greater then specified end_time');
    
    azimuth_non_modulo = azimuth_number*azimuth_resolution + start_azimuth;
    assert(length(azimuth_time) == length(azimuth_non_modulo), 'azimuth_non_modulo vector must be same length as time vector');
    
    %% this modulo has difficulty when the angles are greater than 10^10;
    azimuth_value = mod(azimuth_non_modulo,360);
end