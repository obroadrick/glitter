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
    % show max image and original position of glitter specs
    m = imread('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/imgs/june1/max_june_1.jpg');
    clf;
    % original:
    t = tiledlayout(1,2);
    t.Padding = 'compact';
    t.TileSpacing = 'compact';
    nexttile;
    imagesc(m);hold on;
    colormap(gray);
    msize = int32(.005*(size(m,1)+size(m,2))/2);
    scatter([C(cxs,1)], [C(cxs,2)], msize,'filled');
    scatter(pin(:,1),pin(:,2),msize,'red','filled');
    drawnow;
    % warped: apply homography and show
    nexttile;
    out = transformPointsForward(tform, [C(cxs,1) C(cxs,2)]);
    % find new coordinate of 0,0 after transform to set viewpoint for image
    origin = transformPointsForward(tform, [0 0]);
    % add this origin to all the points for us to show them on top of image
    %outView = imref2d([size(m,1)+origin(1),size(m,2)+origin(2)]);
    %outputImage = imwarp(m,tform,'OutputView',outView)
    outputImage = imwarp(m,tform);
    imagesc(outputImage);hold on;title('here');
    %show where the four known points are
    %imagesc(m);hold on;title('here');
    %scatter(pin(:,1),pin(:,2),'filled');
    %scatter(out(:,1), out(:,2), 'filled');
    drawnow;
    specPos = [out(:,1) out(:,2) zeros(size(out,1),1)];
