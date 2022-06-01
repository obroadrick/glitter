%% finds lighting position which most illuminated each spec
% and use them to 

%% read in the centroids
Cfile = matfile('centroids.mat');
C = Cfile.C;

%% find brightness distributions for all centroids
% (over lighting positions)
% track distributions for each sweep direction
% currently just vertical bars and horizontal bars
tic;
numframes = 256;%max number of frames among the directions
dists = zeros(num,2,numframes);
for direction=1:1
    num = size(C,1);
    x = [1:numframes]';
    i = 0;
    % have to read thru the images to get brightnesses
    for imx = 0:3:765
        i = i + 1; cx = i;
        p = '/Users/oliverbroadrick/Desktop/glitter-stuff/Calibrations5-25/';
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
    x = [1:numframes]';
    means = zeros(size(C,1));
    for ix=1:size(C,1)
        disp(ix);
        %nexttile;
        dist = reshape(dists(ix,direction,:), numframes, 1);
        g = fit(x, dist, 'gauss1');
        mean = g.b1;
        std = g.c1;
        %xline(g.b1); hold on;
        %plot(dist); hold on;
        %title(num2str(ix));
        %plot(g);
        % now try downsampling (d) and fitting just to the spike
        %nexttile;
        k = 4;
        l = int32(mean-k*std);
        r = int32(mean+k*std);
        if l < 1
            l = 1;
        end
        if r > size(dist,1)
            r = size(dist,1);
        end
        distd = dist(l:r);
        xd = [1:size(distd,1)]';
        gd = fit(xd, distd, 'gauss1');
        meand = gd.b1;
        stdd = gd.c1;
        %plot(distd); hold on;
        %xline(meand); hold on;
        %plot(xd, gd(xd));
        diff = mean - (cast(int32(mean-k*std),'like',mean) + meand - 1);
        %title(['difference in means: ' num2str(diff)]);
        means(ix) = meand;% store the zoomed-in mean
    end
end % loop over sweep directions
toc;
%% find vector from light to spec
% map the gaussian means to lighting positions
% get x from vertical bar sweep 
% get y from horizontal bar sweep




