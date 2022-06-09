clear;
%% finds lighting position which most illuminated each spec
% and use them to 

%% read in the centroids (in image coordinates)
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
C = matfile([datap 'image_centroids_2022_06_01.mat']).C;

%% find brightness distributions for all centroids
% (over lighting positions)
% track distributions for each sweep direction
% currently just vertical bars and horizontal bars
tic;
numDirections = 2;
numFrames = [254, 144];
paths = ["/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures V (right - left)/*calib";
         "/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures H (top - bottom)/*calib-h"];
dists = zeros(size(C,1),numDirections,max(numFrames));
means = zeros(numDirections,size(C,1));
for direction=1:numDirections
    num = size(C,1);
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
            % for a quick first take in the interest
            % of getting a glitter characterization done,
            % this just rounds the centroid (floating coords)
            % to nearest ints and checks the single pixel
            % at those integer coords to get the brightness
            % of the centroid... can be easily improved by 
            % moving on to a linear interpolation...
            % will do so only after full characterization pipeline
            % is working to avoid unneeded early complexity
            d1 = int32(C(ix,2));
            d2 = int32(C(ix,1));
            dists(ix, direction, i) = im(d1,d2);
        end
    end
    % fit gaussians to brightness dists and record their means
    x = [1:numFrames(direction)]';
    parfor ix=1:size(C,1)
        if mod(ix, 1000) == 0
            disp([num2str(ix) ' of ' num2str(size(C,1))]);
        end
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
        try
            f = fit(tightx, tight, 'gauss1');
            means(direction, ix) = f.b1;
        catch exception
            % so far, exceptions have only occurred when the 
            % peak is at a boundary which means we don't
            % know whether the true mean should be past the
            % boundary or not and so for now just throw
            % this temporary solution at it to get results
            % but remember to come back and do something
            % about these (like throw them out, find a way
            % to approximate where the mean should be, or
            % decide that this mean is the best we can do and
            % is worth using/keeping around)
            if r == size(dist,1)
                means(direction, ix) = r;
            end
            if l == 1
                means(direction, ix) = 1;
            end
        end
    end
end % loop over sweep directions
% save the means since finding them is the only
% meaningful computational effort
time = datestr(now, 'yyyy_mm_dd');
filename = sprintf('data/lightingmeans_%s.mat',time);
save(filename,'means');
toc;
