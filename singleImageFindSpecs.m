function [C, Cmax, intensitys, intensitysWithArea] = singleImageFindSpecs(im, threshold, options)
    if size(im, 3) == 3
        %convery to grayscale if not already in grayscale
        im = rgb2gray(im);
    end
    % check if options.filter is set to true/false (but if not, filter by
    % default)
    if exist("options.filter", "var")
        if options.filter
            % filter image to keep only specs of glitter bright enough to be of interest
            F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
            %F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
            im = imfilter(im, F);
        end
    else
        % By default, filter
        % filter image to keep only specs of glitter bright enough to be of interest
        F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
        %F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
        im = imfilter(im, F);
    end
    % apply threshold to get binary map with glitter spec regions
    if exist("threshold", "var")
        thresh = threshold;
    else
        thresh = 25;
    end
    Mt = im > thresh;
    % get a list of the region centroids
    numPoints = 1;
    C = [];
    Cmax = [];
    R = regionprops(Mt,'Centroid','PixelIdxList','PixelList','Area');
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
        intensitys(numPoints) = single(intensity);
        areas(numPoints) = single(R(rx).Area);
        numPoints = numPoints + 1;
    end

    % go back through the intensitys and normalize by neighborhood to
    % account for the non-uniform light source (some specs get more light 
    % than others)
    %
    % not so obvious how best to do this... pondering/procrastinating now

    % normalize the intensitys and areas and then multiply them to get a
    % more robust(?) metric for overall sparkle intensity
    
    intensitysWithArea = intensitys ./ max(intensitys);
    areas = areas ./ max(areas);
    intensitysWithArea = intensitysWithArea .* areas;
    
    % TODO ablation testing with this once we get final results (test set
    % and generally working stuff)

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
    hold on; title('Sparkle Intensity (normalized (max intensity)*(area))');
end