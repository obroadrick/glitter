% visualize the checkerboard points that were detected so we can see if
% there are obvious bugs/issues


% Define images to process
numSubsets = 10;

%figure;
%tiledlayout(3,4,"TileSpacing",'tight','Padding','tight');
for i=1:10
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
        imageFileJustNames{fIdx} = allFiles(j).name;
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

    %
    s = ceil(sqrt(size(imageFileNames,2)));
    figure; t= tiledlayout(s,s,'TileSpacing','tight','Padding','tight');
    title(t,dirPath);
    for ix=1:size(imageFileNames,2)
        nexttile;
        hold on;title(imageFileJustNames{ix}(1:size(imageFileJustNames{ix},2)-4),'interpreter','none');
        resizeFactor = .01;
        imagesc(imresize(imread(imageFileNames{ix}),resizeFactor));
        plot(imagePoints(:,1,ix).*resizeFactor,imagePoints(:,2,ix).*resizeFactor,'CX')%,'linewidth',2,'markersize',10);
        %set(gca,'xtick',[],'ytick',[]);%,'xticklabel',[],'yticklabel',[]);
        set(gca,'visible','off','xtick',[],'ytick',[],'YDir','reverse');
        set(findall(gca, 'type', 'text'), 'visible', 'on');
        drawnow;
    end
end