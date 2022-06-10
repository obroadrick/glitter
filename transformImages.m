% transform images with homography

% get paths
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

% get transform
tform = matfile(P.tform).tform;

% read in image
name = '2022-06-09T12,10,34circle-calib-W1127-H574-S48.jpg';
im = imread([P.characterizationTest name]);

% apply transformation
imt = imwarp(im, tform);

% show it all
t = tiledlayout(1,2, 'TileSpacing','tight', 'Padding','t');
num = 16;
t.Padding = 'compact';
t.TileSpacing = 'compact';
nexttile;
imagesc(im);
nexttile;
hImg = imagesc(imt);
%flip y axis to match normal plotting for comparison
hAxs = get(hImg,'Parent');
yAxisTickLabels = get(hAxs, 'YTickLabel');
set(hAxs,'YTickLabel',flipud(yAxisTickLabels));