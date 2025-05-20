function sbet = readSBET(filename)
	nbDoublesInStruct = 17;
	
	fileID = fopen( filename );
	data = fread( fileID, 'float64' );
	fclose( fileID );
	
	nbDoublesInFile = length( data );
	nbPointsInFile = nbDoublesInFile / nbDoublesInStruct;
	
	sbet = ( reshape( data,  nbDoublesInStruct, nbPointsInFile ) ).';
end

% SBET format
% 1 	time;            //timetamp in seconds
% 2 	latitude;        // radians
% 3 	longitude;       // radians
% 4 	ellipsoid_height;// meters
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
