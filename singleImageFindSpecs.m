function [C, Cmax] = singleImageFindSpecs(im)
    if size(im, 3) == 3
        %convery to grayscale if not already in grayscale
        im = rgb2gray(im);
    end
    % filter image to keep only specs of glitter bright enough to be of interest
    F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
    %F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
    imf = imfilter(im, F);
    % apply threshold to get binary map with glitter spec regions
    %thresh = 30;
    thresh = 20;
    %thresh = 150;%for the image i sent in glitter channel
    %thresh = 120;
    Mt = imf > thresh;
    % get a list of the region centroids
    numPoints = 0;
    C = [];
    Cmax = [];
    R = regionprops(Mt,'Centroid','PixelIdxList');
    for rx = 1:size(R,1)
        Pt = R(rx).Centroid;
        numPoints = numPoints + 1;
        C(numPoints,:) = Pt;
        imLike = zeros(size(im));
        imLike(R(rx).PixelIdxList) = im(R(rx).PixelIdxList);
        %maximum = max(max(imLike));
        %[x,y]=find(imLike==maximum);
        [vals, max1] = max(imLike);
        [~, max2] = max(vals);
        Cmax(numPoints,:) = [max2, max1(max2)];
    end
    %disp(thresh);
    %disp(size(C));
end