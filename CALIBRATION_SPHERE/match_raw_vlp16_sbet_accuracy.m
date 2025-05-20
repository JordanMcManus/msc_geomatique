% Boresight and lever arm calibration using spherical targets
%
% Input:
% sphere_struct: struct containing spherical target information
%   nominal_boresight_degrees: 
%       initial values for the boresight angles
%   nominal_lever_arm:
%       initial values for the lever arm
%
% Output:
%   calibrated_boresight_lever_arm:
%       calibrated boresight angles and lever arm
%
% Jordan McManus (2025)
function [sbet_lidar, intensity] = match_raw_vlp16_sbet_accuracy(sbet, accuracy, vlp, vlp_accuracy)
    
    vlp_time = vlp(:,5);
    intensity = vlp(:,4);
    
    beam_number = vlp(:, 9);
    laser_id = beam_number + 1; % 0-15  -->  1-16
    
    % vertical correction
    vertical_correction_table = 0.001*[11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, ...
                                  5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2]'; % VLP-16 user manual p.54-55
    vertical_correction = vertical_correction_table(laser_id);
    
    %% lidar data in spherical coordinates
    azimuth = vlp(:, 6);
    elevation = vlp(:, 7);
    range = vlp(:, 8);
    
    %% convert lidar data to cartesian coordinates
    saz = sind(azimuth);
    caz = cosd(azimuth);
    sel = sind(elevation);
    cel = cosd(elevation);

    x = range.*cel.*saz;
    y = range.*cel.*caz;
    z = range.*sel + vertical_correction;
    
    %% interpolate SBET and SMRMSG to lidar timestamps
    interpSbet = interp1(sbet(:,1), sbet(:,2:end), vlp_time);
    interpAccuracy = interp1(accuracy(:,1), accuracy(:,2:end), vlp_time);
    
    %% lidar and lidar accuracy
    n_ones = ones(size(vlp,1), 1);
    lidar_accuracy = [vlp_accuracy(1)*n_ones, vlp_accuracy(2)*n_ones, vlp_accuracy(3)*n_ones];
    lidar_info = [x, y, z, lidar_accuracy];
    
    %% orientation and orientation accuracy
    orientation_info = [interpSbet(:,7:9), interpAccuracy(:,7:9)];
    
    %% position and position accuracy
    position_info = [interpSbet(:,1:3), interpAccuracy(:,1:3)];
    
    %% format output table
    sbet_lidar = [vlp_time, lidar_info, orientation_info, position_info];
end

% VLP-16 RAW data format
% 1 	xs;            //unit launch vector component x
% 2 	ys;            //unit launch vector component y
% 3 	zs;            //unit launch vector component z
% 4 	intensity;     //beam intensity
% 5 	time;          //lidar timestamp in seconds
% 6 	azimuth;       //beam horizontal angle
% 7 	elevation;     //beam elevation beam angle
% 8 	range;         //beam range
% 9 	beam_id;       //beam id

% SBET NED format
% 1 	time;            //timetamp in seconds
% 2 	xNED;            //northing in meters
% 3 	yNED;            //easting in meters
% 4 	zNED;            //down in meters
% 5 	xSpeed;          //velocity in x direction
% 6 	ySpeed;          //velocity in y direction
% 7 	zSpeed;          //velocity in z direction
% 8 	roll;            //roll angle
% 9 	pitch;           //pitch angle
% 10	heading;         //heading angle
% 11	wander;          //wander
% 12	xForce;          //force in x direction
% 13	yForce;          //force in y direction
% 14	zForce;          //force in z direction
% 15	xAngularRate;	 //angular rate in x direction
% 16	yAngularRate;    //angular rate in y direction
% 17	zAngularRate;    //angular rate in z direction

% Accuracy format
% 1    time;             //timetamp in seconds
% 2    northingSd;       //northing standard deviation
% 3    eastingSd;        //easting standard deviation
% 4    altitudeSd;       //altitude standard deviation
% 5    speedNorthSd;     //northing speed standard deviation
% 6    speedEastSd;      //easting speed standard deviation
% 7    speedAltitudeSd;  //vertical speed standard deviation
% 8    rollSd;           //roll standard deviation
% 9    pitchSd;          //pitch standard deviation
% 10   headingSd;        //heading standard deviation

% Calibration table format
% 1     time;
% 2     xLidar;
% 3     yLidar;
% 4     zLidar;
% 5     xLidarSd;
% 6     yLidarSd;
% 7     zLidarSd;
% 8     rollRad;
% 9     pitchRad;
% 10    headingRad;
% 11    rollSdRad;
% 12    pitchSdRad;
% 13    headingSdRad;
% 14    xPLgf;
% 15    yPgf;
% 16    zPLgf;
% 17    northingSd;
% 18    eastingSd;
% 19    verticalSd;