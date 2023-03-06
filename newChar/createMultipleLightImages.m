% is single-image camera calibration with glitter more feasible when using
% multiple point light sources (rather than one)?

expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct25_nikonz7_35mm/';
num = 4;

figure; tiledlayout(3,3,'Padding','tight','TileSpacing','tight'); colormap('gray');
axes = [];

name = 'blank1';
ambient = rgb2gray(imread([expdir name '.JPG']));
axes(1) = nexttile; imagesc(ambient); title(name);

for ix=1:num
    name1 = [int2str(ix) 'single'];
    im = max(rgb2gray(imread([expdir name1 '.JPG'])) - ambient,0);
    axes(2*ix) = nexttile; imagesc(im); title(name1);
    if ix > 1
        name2 = [int2str(ix) 'all'];
        im = max(rgb2gray(imread([expdir name2 '.JPG'])) - ambient,0);
        axes(2*ix+1) = nexttile; imagesc(im); title(name2);
    end
end


name = 'blank2';
im = imread([expdir name '.JPG']);
axes(num*2+2) = nexttile; imagesc(im); title(name);

linkaxes(axes);