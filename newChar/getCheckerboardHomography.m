function [transformPath, imagePoints, worldPoints] = getCheckerboardHomography(expdir, impath)
    % Given an experiment directory (expdir) and image name in that
    % directory (impath), detect checkerboard points in that image, and
    % compute the homography from world points to image points for points
    % on the checkerboard plane in this image. If this has already been
    % done for this experiment/image, then just return those saved results.
    detector = vision.calibration.monocular.CheckerboardDetector();
    fileName = [expdir impath];
    if ~exist([expdir 'imagePoints.mat'], "file")
        [imagePoints, ~] = detectPatternPoints(detector, fileName);
        save([expdir 'imagePoints'], "imagePoints");
        boardSize = detector.BoardSize;
        save([expdir 'boardSize'], "boardSize");
    else
        imagePoints = matfile([expdir 'imagePoints']).imagePoints;
        boardSize = matfile([expdir 'boardSize']).boardSize;
        detector.BoardSize = boardSize;
    end
    squareSize = 18.25;  % in units of 'millimeters'
    worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);
    tform = estimateGeometricTransform(imagePoints, worldPoints, 'projective');
    tformPath = [expdir 'tform.mat'];
    save(tformPath,"tform");
    transformPath = tformPath;
end