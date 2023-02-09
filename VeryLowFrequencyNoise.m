

figure;
tiledlayout(4,4,'TileSpacing','tight','Padding','tight');
set(gca,'YTickLabel',[],'XTickLabel',[]);
for i=1:16
nexttile;

x = rand(256,256,3);
x = MyBlur(x,30);
x=x(32:256-32,32:256-32,:);
x = x-min(min(min(x)));
x=x ./ max(max(max(x)));
x= imresize(x,[256,256]);
imagesc(x);
set(gca,'YTickLabel',[],'XTickLabel',[]);
end

function blurredRGBImage = MyBlur(rgbImage, sigma)
    % Split into separate color channels.
    [redChannel, greenChannel, blueChannel] = imsplit(rgbImage);
    % Blur each color channel independently.
    blurredR = imgaussfilt(redChannel,sigma);
    blurredG = imgaussfilt(greenChannel,sigma);
    blurredB = imgaussfilt(blueChannel,sigma);
    % Recombine into a single RGB image.
    blurredRGBImage = cat(3, blurredR, blurredG, blurredB);
end