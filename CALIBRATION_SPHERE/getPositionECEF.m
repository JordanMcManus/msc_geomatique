function p = getPositionECEF(pointGeo)
    a = 6378137.0;
    e2 = 0.081819190842622 * 0.081819190842622;
    
    lat = pointGeo(:,1);
    lon = pointGeo(:,2);
    h = pointGeo(:,3);
    
    slat = sin(lat);
    clat = cos(lat);
    
    slon = sin(lon);
    clon = cos(lon);
    
    N = a ./ (sqrt(1 - e2 .* slat .* slat));
    xECEF = (N + h) .* clat .* clon;
    yECEF = (N + h) .* clat .* slon;
    zECEF = (N .* (1 - e2) + h) .* slat;
    
    p = [xECEF, yECEF, zECEF];
end