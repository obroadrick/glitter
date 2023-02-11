
% Define images to process
%expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/';
expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/';
numSubsets = 10;
for i=1:numSubsets
    % Get this batch of file names...:
    dirPath = [expdir num2str(i)];
    allFiles = dir(dirPath);
    allFiles = allFiles(~ismember({allFiles.name},{'.','..'}));
    fIdx = 1;
    for j=1:size(allFiles,1)
        % skip over .mat files
        if ~isempty(regexp(allFiles(j).name,'.mat'))
            %disp(allFiles(j).name)
            continue
        end
        imageFileNames{fIdx} = [allFiles(j).folder '/' allFiles(j).name];
        fIdx = fIdx + 1;00Å¸
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
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', true, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);
    
    % Show how many of the images were used
    disp(sum(imagesUsed));

    % Save parameters
    camParams = cameraParams;
    camParamsErrors = estimationErrors;
    chardir = [dirPath '/'];
    save([chardir 'camParams'], "camParams");
    save([chardir 'camParamsErrors'], "camParamsErrors");

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