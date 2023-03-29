function [transformPath, imagePoints, worldPoints] = getCheckerboardHomography(expdir, imname, other)
    % Given an experiment directory (expdir) and image name in that
    % directory (impath), detect checkerboard points in that image, and
    % compute the homography from world points to image points for points
    % on the checkerboard plane in this image. If this has already been
    % done for this experiment/image, then just return those saved results.
    impath = imname;%old variable renaming
    detector = vision.calibration.monocular.CheckerboardDetector();
    fileName = [expdir impath];
    if isfield(other,'perImageNaming') && other.perImageNaming == true
        if ~exist([expdir imname 'imagePoints.mat'], "file")
            [imagePoints, ~] = detectPatternPoints(detector, fileName);
            if ~(size(imagePoints,1) > 0)
                % points were not detected, but we hold out hope
                % preprocess the image a bit and try again
                im = imread(fileName);
                if size(size(im)) == 3
                    im = imbinarize(rgb2gray(im));
                else
                    im = imbinarize(im);
                end
                [~,nameOnly,~] = fileparts(fileName);
                preprocPath = [expdir nameOnly '_preprocessed.JPG'];
                imwrite(im, preprocPath);
                [imagePoints, ~] = detectPatternPoints(detector, preprocPath);
            end
            save([expdir imname 'imagePoints'], "imagePoints");
            boardSize = detector.BoardSize;
            save([expdir imname 'boardSize'], "boardSize");
        else
            imagePoints = matfile([expdir imname 'imagePoints']).imagePoints;
            boardSize = matfile([expdir imname 'boardSize']).boardSize;
            detector.BoardSize = boardSize;
        end
    else
        if ~exist([expdir 'imagePoints.mat'], "file")
            [imagePoints, ~] = detectPatternPoints(detector, fileName);
            if ~(size(imagePoints,1) > 0 && size(imagePoints,1) < 9*13)
                % points were not detected, but we hold out hope
                % preprocess the image a bit and try again
                im = imgaussfilt(rgb2gray(imread(fileName)),1.5);
                [~,nameOnly,~] = fileparts(fileName);
                preprocPath = [expdir nameOnly '_preprocessed.JPG'];
                imwrite(im, preprocPath);
                [imagePoints, ~] = detectPatternPoints(detector, preprocPath);
            end
            save([expdir 'imagePoints'], "imagePoints");
            boardSize = detector.BoardSize;
            save([expdir 'boardSize'], "boardSize");
        else
            imagePoints = matfile([expdir 'imagePoints']).imagePoints;
            boardSize = matfile([expdir 'boardSize']).boardSize;
            detector.BoardSize = boardSize;
        end
    end
    if size(imagePoints,1) == 0
        disp('No points detected in this image');
        transformPath = [];
        imagePoints = [];
        worldPoints = [];
        return
    end
    squareSize = 18.25;  % in units of 'millimeters'
    worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);
    tform = estimateGeometricTransform(imagePoints, worldPoints, 'projective');
    tformPath = [expdir 'tform.mat'];
    save(tformPath,"tform");
    transformPath = tformPath;
end