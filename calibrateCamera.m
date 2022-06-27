% finds position of camera in the canonical glitter coordinate
% system using (1) matlab checkerboard camera calibration functions,
% (2) images of checkerboards, and (3) homography from image to 
% canonical coordinate system

% inputs: P, a matlab struct containing paths to the necessary images 
%            and homography
function parampath = calibrateCamera(P)
    % read in the homography from image coords to canonical glitter coords
    M = matfile(P.measurements).M;
    %pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];% june 23 xenon captures
    pin = [1606.1387 5375.402; 1294.5402 724.4909; 6527.662 714.07513; 6190.397 5257.4106];%june 27 calibration
    tform = getTransform(P,pin);
    %tform = matfile(P.tform).tform;
    % read in the checkerboard images
    %imsp = P.checkerboardIms;
    %imsp = '/Users/oliverbroadrick/Downloads/Checkerboards-6-23-1_30/';
    imsp = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_27_2020/';
    images = imageSet(imsp);
    imageFileNames = images.Files();
    % find checkerboard points
    I = readimage(images,1); 
    imageSize = [size(I, 1),size(I, 2)];
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    % find bad images
    %imagesNotUsed = imageFileNames(~imagesUsed);
    imagesUsedNames = imageFileNames(imagesUsed);
    %for ix=1:size(imagesNotUsed,1)
    %    delete(imagesNotUsed{ix});
    %end
    %disp(imageFileNames(imagesUsed));
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    % calibrate camera using checkerboard points

    [params, ~, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                    'ImageSize',imageSize);%,"estimateSkew",true);
    displayErrors(estimationErrors, params);
    % save the camera parameters so later with the camera in this position,
    % we can find the location of a new checkerboard in the space without
    % too much trouble (without rerunning the full camera calibration)
    camParams = params;
    camParamsErrors = estimationErrors;
    parampath = [P.data 'camParams_' datestr(now, 'mm_dd_yyyy')];
    save([P.data 'camParams_' datestr(now, 'mm_dd_yyyy')], "camParams");
    save([P.data 'camParamsErrors_' datestr(now, 'mm_dd_yyyy')], "camParamsErrors");
    %% find images with above average mean reprojection error
    errs = params.ReprojectionErrors;
    meanErrs = abs(reshape(mean(mean(errs,1,'omitnan'),2,'omitnan'), size(errs,3), 1));
    % let's look at images whose mean error is more than one standard
    % deviation away from the mean errors of all images
    badImageNames = imagesUsedNames(meanErrs > mean(meanErrs)+std(meanErrs));
    disp('BAD IMAGES:');
    for ix=1:size(badImageNames,2)
        disp(badImageNames(ix));
    end
    %% show camera calibration results
    % plot reprojection errors
    showReprojectionErrors(params);
    % visualize 3d extrinsics 
    figure;
    showExtrinsics(params);
    drawnow;
    % print out estimation errors
    displayErrors(estimationErrors, params);
    % show montage of the images actually being used
    imds = imageDatastore(imsp);
    s = subset(imds, imagesUsed);
    figure;
    montage(s);

    %% get the camera position for this camera calibration
    onGlitterPlane = [imsp 'onGlitterPlane.JPG'];
    [t, R] = findCamPos(P, camParams, onGlitterPlane, pin);
    disp(t);
    disp(R);
end