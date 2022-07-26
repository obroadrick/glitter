% compute and save the homography from images to canonical glitter
% coordinate system using detected fiducial marker positions
% which are currenlty just literals I copy and paste from Addy
% TODO: take marker positions as CSV, etc (a file not literal)
% inputs: P, matlab struct with paths to necessary data (measurements)
function tform = getTransform(P, pin)
    %{
    disp(P);
    disp(P.measurements);
    %}
    M = matfile(P.measurements).M;
    % point correspondences from Addy from fiducial markers (all lower left
    % corners)
    %order: bottom-left, top-left, top-right, bottom-right
    %pout = [0 305; 0 0; 305 0; 305 305];%this is true to image coords for 2d showing    
    %pout = [0 0; 0 M.GLIT_SIDE; M.GLIT_SIDE M.GLIT_SIDE; M.GLIT_SIDE 0];%x,y pairs...this is true to glitter coords for the project
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
end