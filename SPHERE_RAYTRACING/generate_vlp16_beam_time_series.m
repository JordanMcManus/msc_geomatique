function [beam_time, beam_elevation, beam_id, beam_vertical_correction] = generate_vlp16_beam_time_series(start_time, end_time)
    omega_degrees = [-15, 1, -13,  3, -11,  5, -9,  7, ...
                      -7, 9,  -5, 11,  -3, 13, -1, 15]';  % user manual p.54-55
                  
    vertical_correction = 0.001*[11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, ...
                              5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2]'; % user manual p.54-55
                          
	firing_sequence_duration = 55.296e-6; % user manual p.50
    beam_cycle_time = 2.304e-6; % user manual p.50
    
    duration = end_time - start_time;
    
    n = floor(duration/firing_sequence_duration);
    
    beam_index = (0:15)';
    beam_id = repmat(beam_index, n, 1);
    
    beam_number = (0:1:n-1)';
    
    beam_time = beam_id*beam_cycle_time + repelem(beam_number, 16, 1)*firing_sequence_duration + start_time;
    assert(length(beam_time) == length(beam_id), 'length of time array should be the same as length of beam ids');
    
    beam_vertical_correction = repmat(vertical_correction, n, 1);
    assert(length(beam_vertical_correction) == length(beam_id), 'length of beam_vertical_correction angle array should be the same as length of beam ids');
    
    beam_elevation = repmat(omega_degrees, n, 1);
    assert(length(beam_elevation) == length(beam_id), 'length of elevation angle array should be the same as length of beam ids');
end