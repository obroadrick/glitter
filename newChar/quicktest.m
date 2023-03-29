im = imread('/Users/oliverbroadrick/Desktop/glitter-stuff/c2/homography.JPG');
imagePoints = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/c2/imagePoints.mat').imagePoints;
figure;
imagesc(im2bw(imgaussfilt(im,2)));hold on;
plot(imagePoints(:,1),imagePoints(:,2),'mX');
