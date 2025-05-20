function [accuracy5, accuracy10] = readPosPacAccuracy(filename)
    nbDoublesInStruct = 5;
    
    fileID = fopen( filename );
	data = fread( fileID, 'float64' );
	fclose( fileID );

	nbDoublesInFile = length( data );
    assert(mod(nbDoublesInFile, nbDoublesInStruct) == 0, 'nbDoublesInFile must be divisible by nbDoublesInStruct');
	
    nbPointsInFile = nbDoublesInFile / nbDoublesInStruct;
	accuracy5 = ( reshape( data,  nbDoublesInStruct, nbPointsInFile ) ).';
    accuracy10 = [accuracy5(1:2:end,:), accuracy5(2:2:end,:)];
end

% Accuracy format
% 1    double time; //GPS seconds of week
% 2    double northingSd; //northing standard deviation
% 3    double eastingSd; //easting standard deviation
% 4    double altitudeSd; //altitude standard deviation
% 5    double speedNorthSd; //northing speed standard deviation
% 6    double speedEastSd; //easting speed standard deviation
% 7    double speedAltitudeSd; //vertical speed standard deviation
% 8    double rollSd; //roll standard deviation
% 9    double pitchSd; //pitch standard deviation
% 10   double headingSd; //heading standard deviation