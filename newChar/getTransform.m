% compute and save the homography from images to canonical glitter
% coordinate system using detected fiducial marker positions
% which are currenlty just literals I copy and paste from Addy
% TODO: take marker positions as CSV, etc (a file not literal)
% inputs: P, matlab struct with paths to necessary data (measurements)
function tform = getTransform(P, pin)
    M = matfile(P.measurements).M;
    % point correspondences from Addy from fiducial markers (all lower left
    % corners)
    %order: bottom-left, top-left, top-right, bottom-right
    pout = [0 0; 0 M.GLIT_HEIGHT; M.GLIT_WIDTH M.GLIT_HEIGHT; M.GLIT_WIDTH 0];
    % bottom left
    markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                        M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
    pout = pout + markeradjustments;
    if size(pin,1) == 16
        % pout is bottom left corners of fiducial markers in glitter
        % coordinates.. now adjust to get top left, top right, bottom right:
        pout_tl = pout + [0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE];
        pout_tr = pout_tl + [M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0;M.FIDUCIAL_MARKER_SIZE 0];
        pout_br = pout_tr - [0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE;0 M.FIDUCIAL_MARKER_SIZE];
        pout = [pout; pout_tl; pout_tr; pout_br];
    end
    % homography
    tform = fitgeotrans(pin, pout, 'projective');

    % Visualize the homography we found
    vis(pin, pout);
end

function vis(pin, pout)
    figure; 
    tiledlayout(1,2,"TileSpacing","tight","Padding","tight");
    nexttile;title('pin (image points)');
    hold on;set(gca, 'YDir','reverse');
    for ix=1:size(pin,1)
        plot(pin(ix,1), pin(ix,2));
        text(pin(ix,1), pin(ix,2), num2str(ix));
    end
    nexttile;
    hold on;title('pout (world points)');
    for ix=1:size(pout,1)
        plot(pout(ix,1), pout(ix,2));
        text(pout(ix,1), pout(ix,2), num2str(ix));
    end
end