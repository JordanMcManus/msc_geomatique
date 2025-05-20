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

function calibrated_boresight_lever_arm = boresight_lever_arm_calibration(sphere_struct, nominal_boresight_degrees, nominal_lever_arm)
    assert(length(nominal_boresight_degrees) == 3, 'Number of boresight angles should be 3');
    assert(length(nominal_lever_arm) == 3, 'Number of lever arm components should be 3');
    
    Ns = sphere_struct.num_spheres;
    assert(Ns > 0, 'Need at least one sphere');
    
    iteration_counter = 1;
    
    % This inner function calculates the cost as the sum of distances
    % of each point to the known sphere surface.
    % X = [boresight_degrees, lever_arm]
    function cost = f(X)
        % Extract boresight and lever arm for this iteration
        current_boresight_estimation = X(1:3);
        current_lever_arm_estimation = X(4:6);
        
        cost = 0;
        
        % For each sphere target, regeoref and calculate cost (objective)
        for s = 1:Ns
            matched_vlp_data = sphere_struct.matched_vlp_data{s};
            radius = sphere_struct.radius{s};
            center = sphere_struct.center{s};
            georef_sphere = regeoref_ned(matched_vlp_data, current_boresight_estimation, current_lever_arm_estimation);

            % radius is in meters and lidar units are meters so these residuals are meters
            residuals = abs(radius - sqrt(sum(bsxfun(@minus, georef_sphere(:,1:3), center).^2,2)));
            cost = cost + sum(residuals);
        end
        
        disp(['Done iteration: ' num2str(iteration_counter)]);
        iteration_counter = iteration_counter + 1;
    end

    OPTIONS.MaxFunEvals = 5000;
    OPTIONS.MaxIter = 5000;
    initial_X = [nominal_boresight_degrees, nominal_lever_arm];
    calibrated_boresight_lever_arm = (fminsearch(@f, initial_X, OPTIONS));
end