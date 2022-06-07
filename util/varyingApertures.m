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