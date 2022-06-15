function C = singleImageFindSpecs(im)
    if size(im, 3) == 3
        %convery to grayscale if not already in grayscale
        im = rgb2gray(im);
    end
    % filter image to keep only specs of glitter bright enough to be of interest
    F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
    %F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
    imf = imfilter(im, F);
    % apply threshold to get binary map with glitter spec regions
    thresh = 120;
    Mt = imf > thresh;
    % get a list of the region centroids
    numPoints = 0;
    C = [];
    R = regionprops(Mt,'Centroid');
    for rx = 1:size(R,1)
        Pt = R(rx).Centroid;
        numPoints = numPoints + 1;
        C(numPoints,:) = Pt;
    end
end