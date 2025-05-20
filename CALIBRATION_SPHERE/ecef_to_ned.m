function points_ned = ecef_to_ned(pointsECEF, conversion_struct)
    originECEF = conversion_struct.originECEF;
    ecef2ned = conversion_struct.ecef2ned;
    
    points_ned = (ecef2ned*bsxfun(@minus, pointsECEF.', originECEF)).';
end