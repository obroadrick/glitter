% could go through the images here and set isolate just the checkboard
% find largest connected component of white-ish pixels
% find bounding box around that connected component
% set all pixels outside that bounding box to gray
clear;
tic;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
M = matfile([datap 'measurements.mat']).M;
path = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_06_2020/D*';
files = dir(path);
if length(files) < 1
    disp(['no file found at:' path]);
end
t = tiledlayout(8,8);
t.Padding = 'compact';
t.TileSpacing = 'compact';
for ix=1:size(files)
    im = imread([files(ix).folder '/' files(ix).name]);
    nexttile;
    imagesc(im);
    axis off;drawnow;
    % show the largest white-ish connected component
    im = rgb2gray(im);
    whitest = max(max(im,[],2),[],1);
    threshold = whitest*.6;
    nexttile;colormap(gray);
    imagesc(im>threshold);
    axis off;drawnow;
    % show the largest white-ish connected component
    imcc = bwareafilt(im>threshold, 1);
    nexttile;colormap(gray);
    imagesc(imcc);
    axis off;drawnow;
end