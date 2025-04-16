function degree_decimal = dms2dd(degrees, minutes, seconds)
    degree_decimal = degrees + minutes/60 + seconds/3600;
end