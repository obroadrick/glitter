% finds position of camera in the canonical glitter coordinate
% system using (1) matlab checkerboard camera calibration functions,
% (2) images of checkerboards, and (3) homography from image to 
% canonical coordinate system

% inputs: P, a matlab struct containing paths to the necessary images 
%            and homography
function parampath = calibrateCamera(P, fiducialMarkerPoints)
    % read in the homography from image coords to canonical glitter coords
    M = matfile(P.measurements).M;
    %pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];% june 23 xenon captures
    %pin = [1606.1387 5375.402; 1294.5402 724.4909; 6527.662 714.07513; 6190.397 5257.4106];%june 27 calibration
    %{
    allPts = matfile(['/Us '16pts_june23.mat']).arr;
    pin = allPts(1,:);
    pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
    piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
    pin = [pinx' piny'];
    %}
    %pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y 
    %{
    pin = [850.0531005859375	4638.21875;...
            454.743408203125	503.7138366699219;...
            7711.8046875	540.760009765625;...
            7277.14111328125	4664.25];
    %}
    %pin = [865.933837890625	4639.2392578125; 473.364990234375	505.5672302246094; 7731.4736328125	541.7628173828125; 7294.72216796875	4668.791015625];
    %pin = [ 0.7212   4.7309  ;   0.3320   0.5870  ;     7.5774     0.6397  ;     7.1542  4.7480] .* 1000;
    %pin = [718, 4738; 329, 598; 7577, 648; 7153, 4758];
    pin = fiducialMarkerPoints;
    %= [ 0.7212   4.7309  ;   0.3320   0.5870  ;     7.5774     0.6397  ;     7.1542  4.7480] .* 1000;


    tform = getTransform(P,pin);
    %tform = matfile(P.tform).tform;
    % read in the checkerboard images
    imsp = P.checkerboardIms;
    %imsp = '/Users/oliverbroadrick/Downloads/Checkerboards-6-23-1_30/';
    %imsp = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_23_2022/';
    %imsp = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_10_2022';
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

    % TODO / NOTE
    % this is where i could start saving information so that i don't have
    % to go back thru the images (slow) to get it again but can still make
    % new camera calibration estimates for example with and without skew

    % for now avoiding the headache and not doing it partially bc it seems
    % fair to default to using skew from now on and this may only seem like
    % a problem now bc i accidentally ran it without skew last time

    [params, ~, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                    'ImageSize',imageSize,"estimateSkew",true);
    displayErrors(estimationErrors, params);
    % save the camera parameters so later with the camera in this position,
    % we can find the location of a new checkerboard in the space without
    % too much trouble (without rerunning the full camera calibration)
    camParams = params;
    camParamsErrors = estimationErrors;
    parampath = [P.data 'camParams_' datestr(now, 'mm_dd_yyyy')];
    %save([P.data 'camParams__06_10_2022'], "camParams");
    %save([P.data 'camParamsErrors__06_10_2022'], "camParamsErrors");
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
    disp(size(imagesUsedNames));
    translationVectors = params.TranslationVectors;
    norms = vecnorm(translationVectors, 2, 2);
    hist(norms);
    [~, ix] = max(norms);
    disp('max translation distance image has file name:');
    disp(imagesUsedNames(ix));
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
    camPos = t;
    camRot = R;
    P.camPos = [P.data 'camPos_' datestr(now, 'mm_dd_yyyy')];
    P.camRot = [P.data 'camRot_' datestr(now, 'mm_dd_yyyy')];
    save([P.data 'camPos_' datestr(now, 'mm_dd_yyyy')], "camPos");
    save([P.data 'camRot_' datestr(now, 'mm_dd_yyyy')], "camRot");
end