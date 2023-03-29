
% Define images to process
%expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/';
%expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/';
expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/';
numSubsets = 6;
for i=1:numSubsets
    % Get this batch of file names...:
    dirPath = [expdir num2str(i)];
    allFiles = dir(dirPath);
    allFiles = allFiles(~ismember({allFiles.name},{'.','..'}));
    fIdx = 1;
    for j=1:size(allFiles,1)
        % skip over .mat files
        if ~isempty(regexp(allFiles(j).name,'.mat')) || ~isempty(regexp(allFiles(j).name,'DS_Store'))
            %disp(allFiles(j).name)
            continue
        end
        imageFileNames{fIdx} = [allFiles(j).folder '/' allFiles(j).name];
        fIdx = fIdx + 1;
    end

    % Detect calibration pattern in images
    detector = vision.calibration.monocular.CheckerboardDetector();
    if ~isfile([dirPath '/imagePoints.mat'])
        [imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
        save([dirPath '/imagePoints.mat'],"imagePoints");
        save([dirPath '/imagesUsed.mat'],"imagesUsed");
    else
        imagePoints = matfile([dirPath '/imagePoints.mat']).imagePoints;
        imagesUsed = matfile([dirPath '/imagesUsed.mat']).imagesUsed;
    end
    imageFileNames = imageFileNames(imagesUsed);
    
    % Read the first image to obtain image size
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);
    
    % Generate world coordinates for the planar pattern keypoints
    squareSize = 2.445000e+01;  % in units of 'millimeters'
    worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);
    
    % Calibrate the camera
    skew = true;
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', skew, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);
    
    % Show how many of the images were used
    disp(sum(imagesUsed));

    % Save parameters
    camParams = cameraParams;
    camParamsErrors = estimationErrors;
    chardir = [dirPath '/'];

    if skew
        save([chardir 'camParamsSkew'], "camParams");
        save([chardir 'camParamsErrorsSkew'], "camParamsErrors");    
    else
        save([chardir 'camParams'], "camParams");
        save([chardir 'camParamsErrors'], "camParamsErrors");
    end

    % then get campos and camrot from params
    camParams = matfile([chardir 'camParamsSkew.mat']).camParams;
    ApositionIndex = 1;%size(camParams.TranslationVectors,1);
    translationVector = camParams.TranslationVectors(ApositionIndex,:);
    Rvec = camParams.RotationVectors(ApositionIndex,:);
    Rc = rotationVectorToMatrix(Rvec);
    [~, camPos] = extrinsicsToCameraPose(Rc, translationVector);
    camRot = Rc';
    camPosErr = camParamsErrors.ExtrinsicsErrors.TranslationVectorsError(ApositionIndex,:);
    camRotErr = camParamsErrors.ExtrinsicsErrors.RotationVectorsError(ApositionIndex,:);
    if skew
        save([chardir 'camPosSkew.mat'], "camPos");
        save([chardir 'camRotSkew.mat'], "camRot");
        save([chardir 'camPosErrSkew.mat'], "camPosErr");
        save([chardir 'camRotErrSkew.mat'], "camRotErr");
    else
        save([chardir 'camPos.mat'], "camPos");
        save([chardir 'camRot.mat'], "camRot");
        save([chardir 'camPosErr.mat'], "camPosErr");
        save([chardir 'camRotErr.mat'], "camRotErr");
    end
    %{ 
    % other stuff that we don't need to bother showing every time one of
    % these is run
    % View reprojection errors
    h1=figure; showReprojectionErrors(cameraParams);
    
    % Visualize pattern locations
    h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
    
    % Display parameter estimation errors
    displayErrors(estimationErrors, cameraParams);
    
    % For example, you can use the calibration data to remove effects of lens distortion.
    undistortedImage = undistortImage(originalImage, cameraParams);
    
    % See additional examples of how to use the calibration data.  At the prompt type:
    % showdemo('MeasuringPlanarObjectsExample')
    % showdemo('StructureFromMotionExample')
    %}
end