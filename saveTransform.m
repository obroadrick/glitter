function T = saveTransform()
    datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    M = matfile([datap 'measurements.mat']).M;
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
    save([datap 'transform_' datestr(now, 'mm_dd_yyyy')], "tform");
end