function ecef2ned = calculate_ned_matrix(latRad, lonRad)
	
  slat = sin(latRad);
  clat = cos(latRad);
  
  slon = sin(lonRad);
  clon = cos(lonRad);
  
  % NED: North East Down
  ecef2ned = [ -slat*clon, -slat*slon,   clat;...
                  -slon,       clon,       0;...
               -clat*clon,   -clat*slon,   -slat];
end