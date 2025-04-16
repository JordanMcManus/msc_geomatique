function [degrees, minutes, seconds] = dd2dms(dd)
    abs_dd = abs(dd);
    degrees = floor(abs_dd);
    
    res1 = abs_dd - degrees;
    minutes = floor(res1*60);
    
    res2 = res1*60 - minutes;
    
    seconds = res2*60;
end