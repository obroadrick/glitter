P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
specNormals = matfile(P.specNormals).specNormals;

% histograms of surface normal components in each direction (x,y)
figure;
t = tiledlayout(1,2);
t.Padding = 'compact';
t.TileSpacing = 'compact';
ax1 = nexttile(1);
histogram(pi/2-acos(specNormals(:,1)));
xline(mean(pi/2-acos(specNormals(:,1))));
title('Horizontal Components of Spec Surface Normals');
xlabel('Radians');
ax2 = nexttile(2);
histogram(asin(specNormals(:,2)));
xline(mean(asin(specNormals(:,2))));
title('Vertical Components of Spec Surface Normals');
xlabel('Radians');
linkaxes([ax1 ax2], 'xy');