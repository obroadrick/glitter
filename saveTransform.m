% compute and save the homography from images to canonical glitter
% coordinate system using detected fiducial marker positions
% which are currenlty just literals I copy and paste from Addy
% TODO: take marker positions as CSV, etc (a file not literal)
% inputs: P, matlab struct with paths to necessary data (measurements)
function tformpath = saveTransform(P)
    M = matfile(P.measurements).M;
    % point correspondences from Addy from fiducial markers (all lower left
    % corners)
    pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y pairs
    pin = [3035.088 4434.6123; 2557.556 388.33606; 7115.98 307.2471; 7094.5967 5029.888];
    %order: bottom-left, top-left, top-right, bottom-right
    %pout = [0 305; 0 0; 305 0; 305 305];%this is true to image coords for 2d showing    
    pout = [0 0; 0 M.GLIT_SIDE; M.GLIT_SIDE M.GLIT_SIDE; M.GLIT_SIDE 0];%x,y pairs...this is true to glitter coords for the project
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