sparkleResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/sparkleResults').sparkleResults;
checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/checkerResults').checkerResults;

figure;
tiledlayout(2,3);

% camera position
nexttile; hold on;
title('camera position (tx,ty,tz)');
plot3(sparkleResults(:,1),sparkleResults(:,2),sparkleResults(:,3),'r*','MarkerSize',10);
plot3(checkerResults(:,1),checkerResults(:,2),checkerResults(:,3),'gs','MarkerSize',10);
xlabel('x (mm)');ylabel('y (mm)');zlabel('z (mm)');

% camera rotation
nexttile; hold on;
title('rotation (rodrigues pmtrs)');
plot3(sparkleResults(:,4),sparkleResults(:,5),sparkleResults(:,6),'r*','MarkerSize',10);
plot3(checkerResults(:,4),checkerResults(:,5),checkerResults(:,6),'gs','MarkerSize',10);
xlabel('omega1');ylabel('omega2');zlabel('omega3');


% focal length (x,y)
nexttile; hold on;
title('focal length (fx,fy)');
plot(sparkleResults(:,7),sparkleResults(:,8),'r*','MarkerSize',10);
plot(checkerResults(:,7),checkerResults(:,8),'gs','MarkerSize',10);
xlabel('fx (pixels)');ylabel('fy (pixels)');

% optical center (cx,cy)
nexttile; hold on;
title('optical center (cx,cy)');
plot(sparkleResults(:,9),sparkleResults(:,10),'r*','MarkerSize',10);
plot(checkerResults(:,9),checkerResults(:,10),'gs','MarkerSize',10);
xlabel('cx (pixels)');ylabel('cy (pixels)');

% skew
nexttile; hold on;
title('skew');
plot(0,sparkleResults(:,11),'r*','MarkerSize',10);
plot(0,checkerResults(:,11),'gs','MarkerSize',10);
xlabel('nothing at all');ylabel('skew');

nexttile; hold on; title('legend'); 
plot(1.25,1,'r*','MarkerSize',10); 
plot(1.75,1,'gs','MarkerSize',10);
legend('glitter', 'checkerboards')
