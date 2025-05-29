function sbet_launch_vectors = match_launch_vectors_sbet(sbet, launch_vectors)
    % launch_vectors = [time, xl, yl, zl, azimuth, elevation, laser_id, vertical_correction];
    launch_vectors_time = launch_vectors(:, 1); % (n x 1)
    interp = interp1(sbet(:, 1), sbet(:, 2:end), launch_vectors_time);
    position = interp(:, 1:3);
    orientation = interp(:, 7:9);
    sbet_launch_vectors = [launch_vectors_time, position, orientation, launch_vectors(:, 2:end)];
    % sbet_launch_vectors = [time, lat, lon, alt, roll, pitch, heading, launch_vectors(:, 2:end)];
end

% SBET
% 2 	double latitude;	//latitude in radians
% 3 	double longitude;	//longitude in radians
% 4 	double altitude;    //altitude
% 5 	double xSpeed;    	//velocity in x direction
% 6 	double ySpeed;    	//velocity in y direction
% 7 	double zSpeed;    	//velocity in z direction
% 8 	double roll;    	//roll angle
% 9 	double pitch;    	//pitch angle
% 10	double heading;    	//heading angle
% 11	double wander;    	//wander
% 12	double xForce;    	//force in x direction
% 13	double yForce;    	//force in y direction
% 14	double zForce;    	//force in z direction
% 15	double xAngularRate;   	//angular rate in x direction
% 16	double yAngularRate;   	//angular rate in y direction
% 17	double zAngularRate;    //angular rate in z direction