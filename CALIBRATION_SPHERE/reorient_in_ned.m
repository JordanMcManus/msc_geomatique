function reoriented_vectors = reorient_in_ned(roll, pitch, heading, vectors)
    n = size(vectors,1);
    
    assert(size(vectors, 2) == 3, 'Input vectors must be (Nx3)');
    assert(size(roll, 1) == n, 'Input roll must be same length as input vectors');
    assert(size(pitch, 1) == n, 'Input pitch must be same length as input vectors');
    assert(size(heading, 1) == n, 'Input heading must be same length as input vectors');
    
    [ned_dcm_page_1, ned_dcm_page_2, ned_dcm_page_3] = get_ned_dcm_pages(roll, pitch, heading);
    reoriented_vectors =    bsxfun(@times, ned_dcm_page_1, vectors(:, 1)) + ...
                            bsxfun(@times, ned_dcm_page_2, vectors(:, 2)) + ...
                            bsxfun(@times, ned_dcm_page_3, vectors(:, 3));
end