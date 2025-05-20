% Georeferencing in NED
%
% Input:
%       sbet_accuracy_lidar:    table containing matched SBET and laser data
%       boresight_degrees:      boresight angles in degrees
%       lever_arm:              lever arm (INS to optical center)
%
% Output:
%       georef_point_cloud_ned: Georeferenced point cloud in NED LGF
%
% Jordan McManus (2025)

function georef_point_cloud_ned = regeoref_ned(sbet_accuracy_lidar, boresight_degrees, lever_arm)
    
    boresight = deg2rad(boresight_degrees);
    boresightMatrix = get_ned_dcm(boresight);
        
    % Get roll, pitch and heading for lidar series
    roll = sbet_accuracy_lidar(:, 8);
    pitch = sbet_accuracy_lidar(:, 9);
    heading = sbet_accuracy_lidar(:, 10);
    
    % Reorient lever arm in NED
    lever_arm_copies = repmat(lever_arm, size(sbet_accuracy_lidar,1), 1);
    lever_arm_ned = reorient_in_ned(roll, pitch, heading, lever_arm_copies);
    
    % Optical center in NED
    position_reference_point_ned = sbet_accuracy_lidar(:, 14:16);
    optical_center_ned = bsxfun(@plus, position_reference_point_ned, lever_arm_ned);
    
    % Reorient lidar vectors in NED
    lidar_vectors_vlp = sbet_accuracy_lidar(:,2:4);
    lidar_vectors_ins = (boresightMatrix*lidar_vectors_vlp')';
    lidar_vectors_ned = reorient_in_ned(roll, pitch, heading, lidar_vectors_ins);
    
    % georef in NED
    georef_point_cloud_ned = lidar_vectors_ned + optical_center_ned;
end