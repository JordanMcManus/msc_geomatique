function ned_dcm = get_ned_dcm_radians(rph)
  r = rph(1);
  p = rph(2);
  h = rph(3);
  
  sr = sin(r);
  cr = cos(r);
  
  sp = sin(p);
  cp = cos(p);
  
  sh = sin(h);
  ch = cos(h);
  
  % NED
  ned_dcm = [ cp*ch,  sr*sp*ch-cr*sh, sr*sh+cr*sp*ch; ...
              cp*sh,  ch*cr+sr*sp*sh, cr*sp*sh-sr*ch; ...
               -sp,       sr*cp,          cr*cp];
end