function [sbet_ned, conversion_struct] = transform_sbet_to_ned_lgf(sbet)
    
    %% Average position llh = latitude longitude height
    llh_0 = mean(sbet(:,2:4));
    lat_0 = llh_0(1);
    lon_0 = llh_0(2);
    
    %% Transform origin of NED to cartesian coordinates ECEF
    originECEF = getPositionECEF(llh_0).';
    
    %% ECEF to NED matrix
    ecef2ned = calculate_ned_matrix(lat_0, lon_0);
    
    %% Transform trajectory to cartesian coordinates ECEF
    pointsECEF = getPositionECEF(sbet(:,2:4)).';
    
    %% Transform trajectory to cartesian coordinates NWU
    points_ned = (ecef2ned*bsxfun(@minus, pointsECEF, originECEF)).';
    
    %% Express trajectory in NWU
    sbet_ned = [sbet(:,1), points_ned, sbet(:,5:end)];
    
    %% Store conversion data (this allows to convert back to geodetic coordinates)
    conversion_struct.start_timestamp = sbet(1,1);
    conversion_struct.end_timestamp = sbet(end,1);
    conversion_struct.llh_0 = llh_0;
    conversion_struct.originECEF = originECEF;
    conversion_struct.ecef2ned = ecef2ned;
end