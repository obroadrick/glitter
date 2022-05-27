%% let's look at glitter images for various apertures
ims = [];
i = 0;
colormap(gray);
for ix = 0:19
    i = i + 1;
    p = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/diff-apertures/';
    path = [p '*aperture' num2str(ix) '-Glitter.jpg'];
    files = dir(path);
    if length(files) < 1
        disp(['no file found at:' path]);
    end
    if length(files) > 1
        disp(['more than one file at:' path]);
    end
    %disp([files(1).folder '/' files(1).name]);
    disp(ix);
    im = imread([files(1).folder '/' files(1).name]);
    im = double(rgb2gray(im))./255;
    ims(:,:,i) = im;
    imagesc(ims(:,:,i));
    title(i);
    drawnow;
end
%% now show a zoomed in section of glitter specs
t = tiledlayout(3,3, 'Padding', 'none', 'TileSpacing', 'tight'); 
colormap(gray);
y = 1000;
x = 3000;
d = 500;
for ix=1:4:21
    if ix == 21
        ix = 20
    end
    nexttile;
    disp(ix);
    imagesc(ims(y:y+d,x:x+d,ix));
    title(['aperture number ' num2str(ix)]);
    drawnow;
end
% show where on the original image is being shown
for ix=[1 10 20]
    nexttile;
    disp(ix);
    imagesc(ims(:,:,ix)); hold on;
    rectangle('Position', [x y d d], 'EdgeColor', 'r');
    title(['aperture number ' num2str(ix)]);
    drawnow;
end


