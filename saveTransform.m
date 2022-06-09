% compute and save the homography from images to canonical glitter
% coordinate system using detected fiducial marker positions
% which are currenlty just literals I copy and paste from Addy
% TODO: take marker positions as CSV, etc (a file not literal)
% inputs: P, matlab struct with paths to necessary data (measurements)
function tformpath = saveTransform(P)
    M = matfile(P.measurements).M;
    % point correspondences from Addy from fiducial markers (all lower left
    % corners)
    pin = [1571. 5129.;
     1418.  339.;
     6863.  549.;
      6276. 5220.];
    %order: bottom-left, top-left, top-right, bottom-right
    %pout = [0 305; 0 0; 305 0; 305 305];%this is true to image coords for 2d showing    
    pout = [0 0; 0 305; 305 305; 305 0];%this is true to glitter coords for the project
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