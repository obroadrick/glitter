%% finds lighting position which most illuminated each spec
% and use them to 

%% read in the centroids
Cfile = matfile('centroids_2022_06_01.mat');
C = Cfile.C;

%% find brightness distributions for all centroids
% (over lighting positions)
% track distributions for each sweep direction
% currently just vertical bars and horizontal bars
tic;
numDirections = 2;
numFrames = [254, 144];
paths = ['/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures V (right - left)/';
         '/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures H (top - bottom)/'];
dists = zeros(num,numDirections,max(numFrames));
means = zeros(numDirections,size(C,1));
for direction=1:numDirections%was 1:1 whoops
    num = size(C,1);
    x = [1:numFrames(direction)]';
    % have to read thru the images to get brightnesses
    for i=1:numFrames(direction)
        imx = (i-1)*3;
        cx = i;
        p = paths(direction,:);
        path = [p '*calib' num2str(imx) '.0-Glitter.jpg'];
        files = dir(path);
        if length(files) < 1
            disp(['no file found at:' path]);
        end
        if length(files) > 1
            disp(['more than one file at:' path]);
        end
        disp(imx);
        im = rgb2gray(imread([files(1).folder '/' files(1).name]));
        % update each of the dists with this im's data
        for ix=1:size(cxs,1)
            cx = cxs(ix);
            d1 = int32(C(cx,2));
            d2 = int32(C(cx,1));
            dists(ix, direction, i) = im(d1,d2);
        end
    end
    % fit gaussians to brightness dists and record their means
    %t = tiledlayout(uint8(ceil(sqrt(num))),uint8(ceil(sqrt(num))));
    %t.Padding = 'compact';
    %t.TileSpacing = 'compact';
    x = [1:numFrames(direction)]';
    for ix=1:size(C,1)
        disp(ix);
        dist = reshape(dists(ix,direction,:), numFrames(direction), 1);
        g = fit(x, dist, 'gauss1');
        mean = g.b1;
        std = g.c1;
        means(direction, ix) = mean;% store the zoomed-in mean
    end
end % loop over sweep directions
% save the means since finding them is the only
% meaningful computational effort
time = datestr(now, 'yyyy_mm_dd');
filename = sprintf('lightingmeans_%s.mat',time);
save(filename,'means');
toc;
