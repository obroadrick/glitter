%function [C, Cmax, intensitys] = singleImageFindSpecs(im)
    if size(im, 3) == 3
        %convery to grayscale if not already in grayscale
        im = rgb2gray(im);
    end
    % filter image to keep only specs of glitter bright enough to be of interest
    F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
    %F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
    imf = imfilter(im, F);
    % apply threshold to get binary map with glitter spec regions
    thresh = 25;
    Mt = imf > thresh;
    % get a list of the region centroids
    numPoints = 1;
    C = [];
    Cmax = [];
    R = regionprops(Mt,'Centroid','PixelIdxList','PixelList');
    %imLike = zeros(size(im));
    for rx = 1:size(R,1)
        Pt = R(rx).Centroid;
        C(numPoints,:) = Pt;
        %imLike = zeros(size(im));
        %imLike(R(rx).PixelIdxList) = im(R(rx).PixelIdxList);
        [intensity,idx] = max(im(R(rx).PixelIdxList));
        %ix = R(rx).PixelIdxList(location);
        %maximum = max(max(imLike));
        %[x,y]=find(imLike==maximum);
        %[vals, max1] = max(imLike);
        %[~, max2] = max(vals);
        Cmax(numPoints,:) = [R(rx).PixelList(idx,:)];
        %imLike(R(rx).PixelIdxList) = 0;
        intensitys(numPoints) = intensity;
        numPoints = numPoints + 1;
    end

    % go back through the intensitys and normalize by neighborhood to
    % account for the non-uniform light source (some specs get more light 
    % than others)
    %
    % not so obvious how best to do this... pondering/procrastinating now


    %disp(thresh);
    %disp(size(C));
    figure;colormap(gray);
    imagesc(im);hold on;
    plot(C(:,1),C(:,2),'rx');
    %{
    strs = [];
    for i=1:size(intensitys,2)
        strs(i) = convertCharsToStrings(num2str(intensitys(i)));
    end
    %}
    text(C(:,1),C(:,2),string(intensitys),'Color','cyan');
%end