% visualize the checkerboard points that were detected so we can see if
% there are obvious bugs/issues


% Define images to process
numSubsets = 10;
for i=1:numSubsets
    % Get this batch of file names...:
    dirPath = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i)];
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
        fIdx = fIdx + 1;
    end

    % Detect calibration pattern in images
    detector = vision.calibration.monocular.CheckerboardDetector();
    if ~isfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/imagePoints.mat'])
        [imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
        save(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/imagePoints.mat'],"imagePoints");
        save(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/imagesUsed.mat'],"imagesUsed");
    else
        imagePoints = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/imagePoints.mat']).imagePoints;
        imagesUsed = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/imagesUsed.mat']).imagesUsed;
    end
    imageFileNames = imageFileNames(imagesUsed);
end