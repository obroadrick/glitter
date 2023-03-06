%{
sparkleResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/sparkleResults').sparkleResults;
checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/checkerResults').checkerResults;
%}
clear;
casenum = 2;

if casenum == 1
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/sparkleResults').sparkleResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/checkerResults').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/train_after_charMeasOptimization").results;
    intrinsicSparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/train_after_geometricCorrection").results;
    name = '"training" data';
elseif casenum ==2
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/sparkleResults').sparkleResults;
    %checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResults').checkerResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResultsNoSkew').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_charMeasOptimization").results;
    %threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat").results;
    threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults4.mat").results;
    intrinsicSparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_geometricCorrection").results;
    name = '"test" data';
elseif casenum == 3
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/sparkleResults').sparkleResults;
    sparkleResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/test_after_charMeasOptimization').results;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/checkerResults').checkerResults;
    %threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/3dCalibrationResults4.mat").results;
    name = 'third position';
end

figure;
t=tiledlayout(2,3,'TileSpacing','tight','Padding','tight');
title(t,name);

% camera position
nexttile; hold on;
title('camera position (tx,ty,tz)');
if exist("sparkleResults","var")
    plot3(sparkleResults(:,1),sparkleResults(:,2),sparkleResults(:,3),'r*','MarkerSize',10);
end
plot3(sparkleResultsOrig(:,1),sparkleResultsOrig(:,2),sparkleResultsOrig(:,3),'b*','MarkerSize',10);
plot3(checkerResults(:,1),checkerResults(:,2),checkerResults(:,3),'gs','MarkerSize',10);
if exist("threedCalResults", "var")
    plot3(threedCalResults(:,1),threedCalResults(:,2),threedCalResults(:,3),'mX','MarkerSize',10,'LineWidth',2);
end
if exist("intrinsicSparkleResults", "var")
    plot3(intrinsicSparkleResults(:,1),intrinsicSparkleResults(:,2),intrinsicSparkleResults(:,3),'b*','MarkerSize',10,'LineWidth',2);
end
xlabel('x (mm)');ylabel('y (mm)');zlabel('z (mm)');

% camera rotation
nexttile; hold on;
title('rotation (rodrigues pmtrs)');
if exist("sparkleResults","var")
plot3(sparkleResults(:,4),sparkleResults(:,5),sparkleResults(:,6),'r*','MarkerSize',10);
end
plot3(sparkleResultsOrig(:,4),sparkleResultsOrig(:,5),sparkleResultsOrig(:,6),'b*','MarkerSize',10);
plot3(checkerResults(:,4),checkerResults(:,5),checkerResults(:,6),'gs','MarkerSize',10);
if exist("threedCalResults", "var")
    plot3(threedCalResults(:,4),threedCalResults(:,5),threedCalResults(:,6),'mX','MarkerSize',10,'LineWidth',2);
end
if exist("intrinsicSparkleResults", "var")
    plot3(intrinsicSparkleResults(:,4),intrinsicSparkleResults(:,5),intrinsicSparkleResults(:,6),'b*','MarkerSize',10,'LineWidth',2);
end
xlabel('omega1');ylabel('omega2');zlabel('omega3');

% focal length (x,y)
nexttile; hold on;
title('focal length (fx,fy)');
if exist("sparkleResults","var")
plot(sparkleResults(:,7),sparkleResults(:,8),'r*','MarkerSize',10);
end
plot(sparkleResultsOrig(:,7),sparkleResultsOrig(:,8),'b*','MarkerSize',10);
plot(checkerResults(:,7),checkerResults(:,8),'gs','MarkerSize',10);
if exist("threedCalResults", "var")
    plot(threedCalResults(:,7),threedCalResults(:,8),'mX','MarkerSize',10,'LineWidth',2);
end
if exist("intrinsicSparkleResults", "var")
    plot(intrinsicSparkleResults(:,7),intrinsicSparkleResults(:,8),'b*','MarkerSize',10,'LineWidth',2);
end
plot(xlim,xlim,'-b');
xlabel('fx (pixels)');ylabel('fy (pixels)');

% optical center (cx,cy)
nexttile; hold on;
title('optical center (cx,cy)');
if exist("sparkleResults","var")
plot(sparkleResults(:,9),sparkleResults(:,10),'r*','MarkerSize',10);
end
plot(sparkleResultsOrig(:,9),sparkleResultsOrig(:,10),'b*','MarkerSize',10);
plot(checkerResults(:,9),checkerResults(:,10),'gs','MarkerSize',10);
a=size(imread('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/1.JPG'));
plot(a(2)/2, a(1)/2, 'bs','MarkerSize', 10, 'LineWidth', 3);
if exist("threedCalResults", "var")
    plot(threedCalResults(:,9),threedCalResults(:,10),'mX','MarkerSize',10,'LineWidth',2);
end
if exist("intrinsicSparkleResults", "var")
    plot(intrinsicSparkleResults(:,9),intrinsicSparkleResults(:,10),'b*','MarkerSize',10,'LineWidth',2);
end
xlabel('cx (pixels)');ylabel('cy (pixels)');

% skew
nexttile; hold on;
title('skew');
if exist("sparkleResults","var")
plot(0,sparkleResults(:,11),'r*','MarkerSize',10);
end
plot(0,sparkleResultsOrig(:,11),'b*','MarkerSize',10);
plot(0,checkerResults(:,11),'gs','MarkerSize',10);
if exist("threedCalResults", "var")
    plot(0, threedCalResults(:,11),'mX','MarkerSize',10,'LineWidth',2);
end
if exist("intrinsicSparkleResults", "var")
    plot(0, intrinsicSparkleResults(:,11),'b*','MarkerSize',10,'LineWidth',2);
end
xlabel('nothing at all');ylabel('skew');

nexttile; hold on; title('legend'); 
plot(1.2,1,'r*','MarkerSize',10); 
plot(1.4,1,'b*','MarkerSize',10); 
plot(1.6,1,'gs','MarkerSize',10);
plot(1.8,1,'mX','MarkerSize',10,'LineWidth',2);
legend('glitter', 'glitter before i cheated', 'checkerboards', '3d calibration')