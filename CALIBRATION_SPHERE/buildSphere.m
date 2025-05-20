function sphere_points = buildSphere(sphereCenter, knownRadius, numFacesOnSphere)
  [x, y, z] = sphere(numFacesOnSphere);

  x = reshape(x, (numFacesOnSphere+1)*(numFacesOnSphere+1), 1);
  y = reshape(y, (numFacesOnSphere+1)*(numFacesOnSphere+1), 1);
  z = reshape(z, (numFacesOnSphere+1)*(numFacesOnSphere+1), 1);
  rs = knownRadius*[x,y,z];

  % sphere_points = rs + sphereCenter';
  sphere_points = bsxfun(@plus, rs, sphereCenter');
end