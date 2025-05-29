function sbet = readSBET(filename)
    nbDoublesInStruct = 17;
    
    fileID = fopen( filename );
	data = fread( fileID, 'float64' );
	fclose( fileID );

	nbDoublesInFile = length( data );
	nbPointsInFile = nbDoublesInFile / nbDoublesInStruct;
	
	sbet = ( reshape( data,  nbDoublesInStruct, nbPointsInFile ) ).';
end