% compute and save the homography from images to canonical glitter
% coordinate system using detected fiducial marker positions
% which are currenlty just literals I copy and paste from Addy
% TODO: take marker positions as CSV, etc (a file not literal)
% inputs: P, matlab struct with paths to necessary data (measurements)
function tformpath = saveTransform(P)
    M = matfile(P.measurements).M;
    % point correspondences from Addy from fiducial markers (all lower left
    % corners)
    %pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y pairs
    %pin = [3035.088 4434.6123; 2557.556 388.33606; 7115.98 307.2471; 7094.5967 5029.888];
    %{
    pin = [850.0531005859375	4638.21875;...
            454.743408203125	503.7138366699219;...
            7711.8046875	540.760009765625;...
            7277.14111328125	4664.25];
    %}
    %pin = [865.933837890625	4639.2392578125; 473.364990234375	505.5672302246094; 7731.4736328125	541.7628173828125; 7294.72216796875	4668.791015625];
    %pin = [863.2531127929688	4634.79931640625; 472.058349609375	501.9744567871094; 7727.5888671875	543.5863037109375; 7290.06298828125	4667.3486328125];
    %pin = [863.2531127929688	4634.79931640625;472.058349609375	501.9744567871094; 7727.5888671875	543.5863037109375; 7290.06298828125	4667.3486328125];
    pin = [ 0.7212   4.7309  ;   0.3320   0.5870  ;     7.5774     0.6397  ;     7.1542  4.7480] .* 1000;
    

    %order: bottom-left, top-left, top-right, bottom-right]
    %pout = [0 305; 0 0; 305 0; 305 305];%this is true to image coords for 2d showing    
    pout = [0 0; 0 M.GLIT_HEIGHT; M.GLIT_WIDTH M.GLIT_HEIGHT; M.GLIT_WIDTH 0];%x,y pairs...this is true to glitter coords for the project
    %{
    markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                        M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
    %}
    markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                        M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
    pout = pout + markeradjustments;
    % homography
    tform = fitgeotrans(pin, pout, 'projective');
    tformpath = [P.data 'transform.mat'];
    save([P.data 'transform'], "tform");
end