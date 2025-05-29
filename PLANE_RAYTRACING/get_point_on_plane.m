function point_on_plane = get_point_on_plane(plane_general_form)
    point_on_plane = -plane_general_form(4)*plane_general_form(1:3);
end