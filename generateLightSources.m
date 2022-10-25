clear;

% generate images to be used as light sources
path = '/Users/oliverbroadrick/Desktop/multiplePointLights/';

w = 3840;
h = 2160;

xoff = 500;
yoff = 300;

positions = [xoff yoff; w-xoff h-yoff; xoff h-yoff; w-xoff yoff];

r = 9;

num_positions = size(positions,1);
Amax = zeros(w,h);

% prepare to plot all the images we create
figure; tiledlayout(3,3,'TileSpacing','tight','Padding','tight'); colormap('gray');

% first save an image with no light sources yet
name = 'blackScreen';
imwrite(Amax', [path name '.png']);

for n=1:size(positions,1)
    % CREATE AN ARRAY A_NEW WITH THE NEW LIGHT POSITION AND ADD IT TO THE
    % CUMULATIVE ARRAY A_MAX
    Anew = zeros(w,h);
    x = positions(n,1);
    y = positions(n,2);
    for jx=1:r
        for kx=1:r
            Anew(x+jx,y+kx) = 255;% set to white
        end
    end
    Amax = max(Amax, Anew);

    % SHOW AND SAVE THE IMAGES
    % save the image with the nth light
    name1 = [int2str(n) 'thPositionOnly'];
    imwrite(Anew', [path name1 '.png']);
    nexttile; imagesc(Anew); title(name1);
    if n>1
        % save the image with the first n lights
        name2 = [int2str(n) 'CumulativePositions'];
        imwrite(Amax', [path name2 '.png']);
        nexttile; imagesc(Amax); title(name2);
    end
end

