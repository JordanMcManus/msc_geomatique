function points_ned = transform_geo_to_ned(points_geo, conversion_struct)
    
    %% Origin of NED expressed in cartesian coordinates ECEF
    originECEF = conversion_struct.originECEF;
    
    %% ECEF to NED matrix
    ecef2ned = conversion_struct.ecef2ned;
    
    %% Transform points from geographic to cartesian coordinates ECEF
    pointsECEF = getPositionECEF(points_geo).';
    
    %% Transform points from ECEF to cartesian coordinates NED
    points_ned = (ecef2ned*bsxfun(@minus, pointsECEF, originECEF)).';
end