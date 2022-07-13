% let's get (just from the max image)the max brightness of each
% characterized spec.
% and then this can be used to scale the brightness of observed sparkles
% according to each specs own unique max brightness.

function maxbpath = maxBrightnesses(P)
    M = matfile(P.measurements).M;
    
    m = rgb2gray(imread(P.maxImage));
    figure;imagesc(m);colormap(gray);drawnow;
    
    %% use image centroids (rather than the canonical centroids) from the
    % characterization
    imageCentroids = matfile(P.imageCentroids).imageCentroids;
    
    %% find max brightness (linearly interpolated) at these centroids
    maxBrightness = zeros(size(imageCentroids,1),1);
    %for ix=1:size(imageCentroids)
    %    maxBrightness(ix) = interp2(m,imageCentroids(ix,1),imageCentroids(ix,2));
    %end
    maxBrightness(:) = interp2(double(m),imageCentroids(:,1)',imageCentroids(:,2)');
    
    %% save results
    maxbpath = [P.data 'maxBrightness.mat'];
    save(sprintf([P.data 'maxBrightness_%s'],datestr(now, 'mm_dd_yyyy')), "maxBrightness");
end
