function pout = getFiducialMarkerPts()
    % computes and returns the coordinates of the 16 corners of the
    % fiducial markers in the canonical glitter coordinate system

    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    M = matfile(P.measurements).M;
    pout = [0 0; 0 M.GLIT_HEIGHT; M.GLIT_WIDTH M.GLIT_HEIGHT; M.GLIT_WIDTH 0];
    % bottom left
    markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                        M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
    pout = pout + markeradjustments;
    % pout is bottom left corners of fiducial markers in glitter
    % coordinates.. now adjust to get top left, top right, bottom right:
    pout_tl = pout + [0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE];
    pout_tr = pout_tl + [M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0];
    pout_br = pout_tr - [0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE];
    pout = [pout; pout_tl; pout_tr; pout_br];
end