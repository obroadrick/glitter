input = struct('name',['Nikon Z7 (February 16 Data)'], ...
                'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/feb16/', ...
                'lightPosFname', ['lightPos1.mat'], ...
                'sixteen', true, ...
                'numLightPositions', 1);
input = struct('name',['Nikon Z7 (February 21 Data)'], ...
                'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/feb21/', ...
                'lightPosFname', ['lightPos1.mat'], ...
                'sixteen', true, ...
                'numLightPositions', 1);

input = struct('name',['Nikon Z7 (February 21 Data)'], ...
                'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/feb21-constant-shutter/', ...
                'lightPosFname', ['lightPos1.mat'], ...
                'sixteen', true, ...
                'numLightPositions', 1);

% Get file names
dirPath = [input.expdir];
allFiles = dir(dirPath);
allFiles = allFiles(~ismember({allFiles.name},{'.','..'}));
fIdx = 1;
for j=1:size(allFiles,1)
    % skip over .mat files
    if ~isempty(regexp(allFiles(j).name,'.mat')) || ~isempty(regexp(allFiles(j).name,'DS_Store'))
        continue
    end
    if ~isempty(regexp(allFiles(j).name,'Homography'))
        continue
    end
    imageFileNames{fIdx} = [allFiles(j).folder '/' allFiles(j).name];
    imageFileJustNames{fIdx} = [allFiles(j).name];
    fIdx = fIdx + 1;
end
set(0,'DefaultFigureVisible','off');
showIms = false;
if showIms
% Show images we're working with here
s = ceil(sqrt(size(imageFileNames,2)));
figure; t= tiledlayout(s,s,'TileSpacing','tight','Padding','tight');
title(t,dirPath);
for ix=1:size(imageFileNames,2)
    disp(imageFileNames{ix})
    axes(ix) = nexttile;
    hold on;title(imageFileJustNames{ix}(1:size(imageFileJustNames{ix},2)-4),'interpreter','none');
    resizeFactor = .01;
    %imagesc(imresize(imread(imageFileNames{ix}),resizeFactor));
    imagesc(imread(imageFileNames{ix}));
    %plot(imagePoints(:,1,ix).*resizeFactor,imagePoints(:,2,ix).*resizeFactor,'CX')%,'linewidth',2,'markersize',10);
    %set(gca,'xtick',[],'ytick',[]);%,'xticklabel',[],'yticklabel',[]);
    set(gca,'visible','off','xtick',[],'ytick',[],'YDir','reverse');
    set(findall(gca, 'type', 'text'), 'visible', 'on');
    drawnow;
end
linkaxes(axes);
end % end if showIms

%% for the first image, get the locations of the specs which are ransac inliers
index = 1;
expdir = input.expdir;
impath = imageFileNames{index};
disp(impath);
%%
lightPos = matfile([expdir input.lightPosFname]).lightPos;
%skew = input.skew; % whether we assume zero skew
skew=false;
compare = false; % whether this script will output comparisons with checkerboard results
%fprintf('\n%s\n',input.name);
% Find ArUco markers (if they haven't been detected and saved already)
homographyImpath = [expdir 'EHomography.jpg'];
if ~isfile([expdir '16pts.mat'])
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', homographyImpath, [expdir '16pts.mat']);
    system(cmd);
end
pin = loadPoints([expdir '16pts.mat'], input.sixteen);
% Get characterization paths for spec locations and surface normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
%{
% Estimate translation
other.inlierThreshold = 25;
ambientImage = -1;
other.compare = false;
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);
reflectedRayToPinholeDists{index} = other.reflectedRayToPinholeDists;
mostInliersIntensitys{index} = other.mostInliersIntensitys;
%}
% Find all the specs (call them mostInliersImageSpecPos to avoid renaming
% in the rest of this file).
options.filter = false;
[mostInliersImageSpecPos, ~, intensitys, intensitysWithArea] = singleImageFindSpecs(imread(impath), 10, options);
mostInliersIntensitys{index} = intensitys';

%% get the rest of the brightnesses based on the previous mostInliers image spec locations
for index=1:size(imageFileNames,2)

expdir = input.expdir;
impath = imageFileNames{index};
lightPos = matfile([expdir input.lightPosFname]).lightPos;
%skew = input.skew; % whether we assume zero skew
skew = false;
compare = false; % whether this script will output comparisons with checkerboard results
%fprintf('\n%s\n',input.name);

%%
im = imread(impath);
% get the brightness of sparkles
options.filter = false;
[C, Cmax, intensitys] = singleImageFindSpecs(im, 10, options);

% get the 'matching' specs by taking the nearest spec in the image but only
% if it's within 20 pixels (since beyond that is almost certainly another
% spec).. if there isn't one near enough, the new intensity is zero
[idx, dist] = knnsearch(C, mostInliersImageSpecPos);
for i=1:size(idx)
    if dist(i) >= 20
        curIntensity = 0;
    else
        curIntensity = intensitys(idx(i));
    end
    mostInliersIntensitys{index}(i) = curIntensity;
end

% print an update
fprintf('image %d: %d specs found\n', index, size(C,1));

end % end for loop

%{
% visualize the histograms of reflectedRayToPinholeDists for each index
figure;
w=ceil(sqrt(2)*sqrt(size(reflectedRayToPinholeDists,2)));
tiledlayout(w,w,'TileSpacing','tight','Padding','tight');
for index=1:size(reflectedRayToPinholeDists,2)
    nexttile;
    histogram(reflectedRayToPinholeDists{index},10);
    xlabel('mm to pinhole');
    xlim([0 other.inlierThreshold]);
    nexttile
    histogram(mostInliersIntensitys{index},15);
    xlabel('itensity');
    xlim([0 1]);
end

%%
% intensity vs distance to pinhole
figure;
w=ceil(sqrt(size(reflectedRayToPinholeDists,2)));
tiledlayout(w,w,'TileSpacing','tight','Padding','tight');
for index=1:size(reflectedRayToPinholeDists,2)
    nexttile;
    plot(reflectedRayToPinholeDists{index}, mostInliersIntensitys{index},'b*');
    xlabel('mm to pinhole');
    ylabel('intensity');
    xlim([0 other.inlierThreshold]);
    ylim([0 1]);
    title(imageFileJustNames(index));
end
%}
%%
set(0,'DefaultFigureVisible','on');
% get intensity distribution over aperture for each inlier spec
intensitysOverApertures = [];
for apertureIndex=1:size(mostInliersIntensitys,2)
    for specIndex=1:size(mostInliersIntensitys{1},1)
        intensitysOverApertures(apertureIndex, specIndex) = mostInliersIntensitys{apertureIndex}(specIndex);
    end
end

%%
% intensity vs aperture
%apertures = [1.8 2.2 2.5 3.2 3.5 4 4.5 5 5.6 6.3 7.1 8 9 10 11 13 14 16];
% get apertures from file names
apertureConversionArray = [1.8 2 2.2 2.5 2.8 3.2 3.5 4 5 5.6 6.3 7.1 8 9 10 11 13 14 16 30];
shutterSpeedConversionArray= [-1 -1 -1 30 25 20 15 13 10 8 6 5 4 3 2.5 2 1.6 1.3 1 1/1.3 1/1.6 1/2 1/2.5 1/3 1/4 1/5 1/6 1/8 1/10 1/13 1/15 1/20 1/25 1/30 1/40 1/50 1/60 1/80];
for i=1:size(imageFileNames,2)
    a = extractBetween(imageFileJustNames{i},'a','_s');
    apertureIndex = str2num(a{1}) + 1;
    a
    b = extractBetween(imageFileJustNames{i},'_s','.jpg');
    shutterSpeedIndex = str2num(b{1}) + 1;
    apertures(i) = apertureConversionArray(apertureIndex);
    shutterSpeeds(i) = shutterSpeedConversionArray(shutterSpeedIndex);
end

%%
% sort by brightest specs first
[~,inx]=sort(intensitysOverApertures(1,:), 'descend');
intensitysOverAperturesSorts = intensitysOverApertures(:,inx);

%% 
% sort rows by aperture
[~,ix]=sort(apertures);
intensitysOverAperturesSortsSorts = intensitysOverAperturesSorts(ix,:);
%% 
% plot the brightest n specs' intensities across the apertures
figure;
hold on;
for i=1:size(intensitysOverApertures,2)%size(intensitysOverAperturesSorts,2)
    plot(sort(apertures), intensitysOverAperturesSortsSorts(:,i),'x-');
    hold on;
    xlabel('aperture f-x');
    ylabel('intensity');
    xlim([1 16]);
    %ylim([0 255]);
end


%% 
% now normalize by the brightest pixel
for i=1:size(intensitysOverApertures,1)%size(intensitysOverAperturesSorts,2)
    intensitysOverAperturesSortsSorts(i,:) = intensitysOverAperturesSortsSorts(i,:) ./ max(intensitysOverAperturesSortsSorts(i,:));
end
%% 
% plot the brightest n specs' intensities across the apertures
figure;
hold on;
for i=1:size(intensitysOverApertures,2)%size(intensitysOverAperturesSorts,2)
    plot(sort(apertures), intensitysOverAperturesSortsSorts(:,i),'x-');
    hold on;
    xlabel('aperture f-x');
    ylabel('intensity');
    xlim([1 16]);
    %ylim([0 255]);
end
title('intensity vs aperture (intensity normalized by max intensity at each aperture')

%%
% fingerprint for a given aperture
% after normalizing (to account for difference in light source intensity
% and shutterspeed, perhaps more importantly shutterspeed, and I guess also
% exposure in general maybe including ISO and more..)
% we then can compute the average intensity of the brightest N specs. if
% all specs are bright then the aperture must be large but if only some are
% bright and other specs relatively are dim but still sparkling then the
% aperture is small...?
% to see if this has any potential, let's plot the average normalized
% intensity for each aperture:

% TODO: NORMALIZE AND FIND AVERAGE BUT ONLY FOR SPECS YOU ACTUALLY SEE (IE 
% ONES THAT ARE ABOVE 0 IN INTENSITY FOR THAT IMAGE/APERTURE
figure;
hold on;
avgNormalizedIntensity = (sum(intensitysOverAperturesSortsSorts,2)./size(intensitysOverAperturesSortsSorts,2));
plot(sort(apertures), avgNormalizedIntensity,'x-');
hold on;
xlabel('aperture f-x');
ylabel('avg normalized intensity');
xlim([1 16]);
%ylim([0 255]);
title('avg normalized intensity vs aperture (intensity normalized by max intensity at each aperture')

