%% finds lighting position which most illuminated each spec
% and use them to 

%% read in the centroids
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
C = matfile([datap 'centroids_2022_06_01.mat']).C;

%% find brightness distributions for all centroids
% (over lighting positions)
% track distributions for each sweep direction
% currently just vertical bars and horizontal bars
tic;
numDirections = 2;
numFrames = [254, 144];
paths = ["/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures V (right - left)/*calib";
         "/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures H (top - bottom)/*calib-h"];
dists = zeros(num,numDirections,max(numFrames));
means = zeros(numDirections,size(C,1));
for direction=1:numDirections
    num = size(C,1);
    x = [1:numFrames(direction)]';
    % have to read thru the images to get brightnesses
    for i=1:numFrames(direction)
        imx = (i-1)*3;
        cx = i;
        p = paths(direction,:);
        path = [convertStringsToChars(p) num2str(imx) '.0-Glitter.jpg'];
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
        for ix=1:size(C,1)
            d1 = int32(C(ix,2));
            d2 = int32(C(ix,1));
            dists(ix, direction, i) = im(d1,d2);
        end
    end
    % fit gaussians to brightness dists and record their means
    x = [1:numFrames(direction)]';
    for ix=1:size(C,1)
        disp([num2str(ix) ' of ' num2str(size(C,1))]);
        dist = reshape(dists(ix,direction,1:numFrames(direction)), numFrames(direction), 1);
        [peak peakidx] = max(dist);
        l = int32(peakidx-10);
        r = int32(peakidx+10);
        if l < 1
            l = 1;
        end
        if r > size(dist,1)
            r = size(dist,1);
        end
        tight = dist(l:r);
        tightx = x(l:r);
        f = fit(tightx, tight, 'gauss1');
        mean = f.b1;
        disp(mean);
        means(direction, ix) = mean;% store the zoomed-in mean
    end
end % loop over sweep directions
% save the means since finding them is the only
% meaningful computational effort
time = datestr(now, 'yyyy_mm_dd');
filename = sprintf('lightingmeans_%s.mat',time);
save(filename,'means');
toc;
