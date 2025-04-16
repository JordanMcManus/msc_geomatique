function ned_dcm = get_ned_dcm_degrees(rph)
  r = rph(1);
  p = rph(2);
  h = rph(3);
  
  sr = sind(r);
  cr = cosd(r);
  
  sp = sind(p);
  cp = cosd(p);
  
  sh = sind(h);
  ch = cosd(h);
  
  % NED
  ned_dcm = [ cp*ch,  sr*sp*ch-cr*sh, sr*sh+cr*sp*ch; ...
              cp*sh,  ch*cr+sr*sp*sh, cr*sp*sh-sr*ch; ...
               -sp,       sr*cp,          cr*cp];
end