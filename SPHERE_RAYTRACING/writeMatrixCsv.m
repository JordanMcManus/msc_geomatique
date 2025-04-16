%Écriture d'une matrice dans fichier
function writeMatrixCsv(filename, data)
    numColumns = size(data,2);
    
    format = ['%.8f'];
    for i=1:(numColumns-1)
        format = [format ',%.8f'];
    end
    format = [format '\n'];

    fileID = fopen(filename,'w');
    nbytes = fprintf(fileID,format,data.');
    fclose(fileID);
end