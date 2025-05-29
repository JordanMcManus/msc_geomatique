function plane_general_form = get_plane_general_form(plane_coefficients)
    % plane coefficients:
    % Ax + By + Cz + D = 0
    A = plane_coefficients(1);
    B = plane_coefficients(2);
    C = plane_coefficients(3);
    D = plane_coefficients(4);
    
    sf = 1 / norm([A,B,C]);
    a = sf*A;
    b = sf*B;
    c = sf*C;
    d = sf*D;
    
    % plane general form
    % ax + by + cz + d = 0
    % with a*a + b*b + c*c = 1
    plane_general_form = [a, b, c, d];
end