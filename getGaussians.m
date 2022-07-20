% fits gaussians (storing their means) to the brightness
% distribution of specs of glitter from a full sweep in 
% each direction, using stored spec centroid locations

% inputs: P, matlab struct with paths to the necessary data/images
function meansPath = getGaussians(P)
    %% finds lighting position which most illuminated each spec
    % and use them to 
    
    %% read in the centroids (in image coordinates)
    C = matfile(P.imageCentroids).imageCentroids;
    Ccanonical = matfile(P.canonicalCentroids).canonicalCentroids;
    
    %% find brightness distributions for all centroids
    % (over lighting positions)
    % track distributions for each sweep direction
    % currently just vertical bars and horizontal bars
    numDirections = 2;
    numFrames = [size(dir(P.leftRightSweep),1)-2,size(dir(P.upDownSweep),1)-2];
    %253, 144];
    paths = [convertCharsToStrings(P.leftRightSweep);...
               convertCharsToStrings(P.upDownSweep)];
    regexs = ["*calib", "*calib-h"];
    dists = zeros(size(C,1),numDirections,max(numFrames));
    means = [];%zeros(numDirections,size(C,1));
    %%
    for direction=1:numDirections
        num = size(C,1);
        % read thru the images to get brightnesses
        for i=1:numFrames(direction)
            imx = i-1;%(i-1)*3;
            cx = i;
            p = convertStringsToChars(paths(direction));
            r = convertStringsToChars(regexs(direction));
            path = [p r num2str(imx) '.0-Glitter.JPG'];
            files = dir(path);
            if length(files) < 1
                disp(['no file found at:' path]);
            end
            if length(files) > 1
                disp(['more than one file at:' path]);
            end
            im = rgb2gray(imread([files(1).folder '/' files(1).name]));
            % update each of the dists with this im's data
            for ix=1:size(C,1)
                d1 = int32(C(ix,2));
                d2 = int32(C(ix,1));
                % find max within the 8-connected component of the
                % centroid (could modify to interpolation or larger max)
                dists(ix, direction, i) = max([im(d1,d2),...
                                               im(d1+1,d2+1),...
                                               im(d1+1,d2-1),...
                                               im(d1+1,d2),...
                                               im(d1,d2+1),...
                                               im(d1,d2-1),...
                                               im(d1-1,d2+1),...
                                               im(d1-1,d2-1),...
                                               im(d1-1,d2)]);
            end
        end
        %%
        % fit gaussians to brightness dists and record their means
        x = [1:numFrames(direction)]';
        parfor ix=1:size(C,1)
            dist = reshape(dists(ix,direction,1:numFrames(direction)), numFrames(direction), 1);
            [~, peakidx] = max(dist);
            l = int32(peakidx-10);
            r = int32(peakidx+10);
            if l < 1 || r > size(dist,1)
                % if past an edge, throw out
                means(direction, ix) = -1;
                stds(direction, ix) = -1;
                continue
            end
            tight = dist(l:r);
            tightx = x(l:r);
            try
                f = fit(tightx, tight, 'gauss1');
                means(direction, ix) = f.b1;
                stds(direction, ix) = f.c1;%standard deviation
            catch exception
                % if issue, throw out with a -1 marker 
                % (happens rarely and at the edges)
                means(direction, ix) = -1;
                stds(direction, ix) = -1;
            end
        end
    end % loop over sweep directions

    % now keep only the means/stds that gave reasonable results (i.e. those
    % which did not get assigned a -1 in the end)
    newmeans = [];
    newstds = [];
    for direction=1:numDirections
        curmeans = means(direction,:);
        newmeans(direction,:) = curmeans(curmeans ~= -1);
        curstds = stds(direction,:);
        newstds(direction,:) = curstds(curstds ~= -1);
        newC = C(curmeans ~= -1,:);
        newCcanonical = Ccanonical(curmeans ~= -1,:);
        disp(sum(curmeans ~= -1));
        disp(sum(curstds ~= -1));
    end
    means = newmeans;
    stds = newstds;
    imageCentroids = newC;
    canonicalCentroids = newCcanonical;

    % save the means since finding them is the only
    % meaningful computational effort
    filename = sprintf([P.data 'lightingmeans_%s'],datestr(now, 'mm_dd_yyyy'));
    meansPath = filename;
    save(filename,'means');
    filename = sprintf([P.data 'lightingstds_%s'],datestr(now, 'mm_dd_yyyy'));
    save(filename,'stds');
    % also save the corresponding new centroids
    imageCentroidsPath = [P.data 'image_centroids_' datestr(now, 'mm_dd_yyyy')];
    canonicalCentroidsPath = [P.data 'canonical_centroids_' datestr(now, 'mm_dd_yyyy')];
    save(imageCentroidsPath, "imageCentroids");
    save(canonicalCentroidsPath, "canonicalCentroids");
end