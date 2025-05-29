function georef_vector = georef_time_independant_and_frame_independant(beam_vector, prp_vector, orientation_matrix, boresight_matrix, lever_arm_vector)
%georef_time_independant_and_frame_independant   Beam Georeferencing.
%   Georeferencing equation for beam data.
%   Time independent.
%   Frame independent.
%
%   Time independent means that time is not an input.
%
%   Frame independent means that vectors and matrices are already expressed
%   according to valid coordinate conventions.
%
%   GIGO: If all coordinate conventions are respected
%   and if the provided data is coherent
%   then this function provides a meaningful result.
%
%   beam_vector        - Cartesian vector of beam in LiDAR frame
%   prp_vector         - Body position in reference frame
%   orientation_matrix - Body orientation in reference frame
%   boresight_matrix   - Alignment matrix between LiDAR and Body
%   lever_arm_vector   - Position of LiDAR optical center in Body frame
%
%
%   georef_vector      - Georeferencing result
%
%   Jordan McManus 2024

    assert(size(beam_vector, 1) == 3, 'Beam vector must be (3x1)');
    assert(size(beam_vector, 2) == 1, 'Beam vector must be (3x1)');
    
    assert(size(prp_vector, 1) == 3, 'Position reference point must be (3x1)');
    assert(size(prp_vector, 2) == 1, 'Position reference point must be (3x1)');
    
    assert(size(lever_arm_vector, 1) == 3, 'Lever arm must be (3x1)');
    assert(size(lever_arm_vector, 2) == 1, 'Lever arm must be (3x1)');
    
    assert(size(orientation_matrix, 1) == 3, 'Orientation matrix must be (3x3)');
    assert(size(orientation_matrix, 2) == 3, 'Orientation matrix must be (3x3)');
    
    assert(size(boresight_matrix, 1) == 3, 'Boresight matrix must be (3x3)');
    assert(size(boresight_matrix, 2) == 3, 'Boresight matrix must be (3x3)');
    
    eps_proper_rot = 1e-6;
    assert(abs(det(orientation_matrix) - 1) < eps_proper_rot, 'Orientation matrix must be a proper rotation');
    assert(abs(det(boresight_matrix) - 1) < eps_proper_rot, 'Boresight matrix must be a proper rotation');
    
    georef_vector = prp_vector + orientation_matrix*(boresight_matrix*beam_vector + lever_arm_vector);
    
    assert(size(georef_vector, 1) == 3, 'Georef vector must be (3x1)');
    assert(size(georef_vector, 2) == 1, 'Georef vector must be (3x1)');
end