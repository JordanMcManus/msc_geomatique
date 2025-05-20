function [ned_dcm_page_1, ned_dcm_page_2, ned_dcm_page_3] = get_ned_dcm_pages(roll, pitch, heading)
  sr = sin(roll); 
  cr = cos(roll);
  
  sp = sin(pitch);
  cp = cos(pitch);
  
  sh = sin(heading);
  ch = cos(heading);
  
  m11 = cp.*ch; % (Nx1)
  m21 = cp.*sh; % (Nx1)
  m31 = -sp;    % (Nx1)
  
  m12 = sr.*sp.*ch - cr.*sh; % (Nx1)
  m22 = ch.*cr + sr.*sp.*sh; % (Nx1)
  m32 = sr.*cp;              % (Nx1)
  
  m13 = sr.*sh + cr.*sp.*ch; % (Nx1)
  m23 = cr.*sp.*sh - sr.*ch; % (Nx1)
  m33 = cr.*cp;              % (Nx1)
  
  ned_dcm_page_1 = [ m11, m21, m31 ];
  ned_dcm_page_2 = [ m12, m22, m32 ];
  ned_dcm_page_3 = [ m13, m23, m33 ];
  
  % NED
%   ned_dcm = [ cp*ch,  sr*sp*ch-cr*sh, sr*sh+cr*sp*ch; ...
%               cp*sh,  ch*cr+sr*sp*sh, cr*sp*sh-sr*ch; ...
%                -sp,       sr*cp,          cr*cp];
end