% play with and visualize the standard deviations in sparkle brightness
% distributions over varying light source positions in each of the x and y
% directions... helpful for thinking about the receptive field of the
% glitter specs (aka is symmetric usually, does it vary much in size across
% specs, etc...)

stds_orig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/check_lightingstds_07_18_2022.mat').stds';
% it should be noted that "Sparkle Deviations" is a good name for a band

% the unit of these standard deviations in frame index

%% show x std vs y std color coded by density/frequency 
% (to make it a more readable 2d plot instead of the 3d histogram 
% alternative that i started with)
sums = sum(stds_orig,2);
keep = sums < 20;
stds = stds_orig(keep,:);

nbins = 100;
N = hist3(stds, 'nbins', [nbins,nbins]);

[x,y] = ndgrid(1:nbins);
scatter(x(:),y(:),[],N(:),'filled');
colorbar;
colormap jet;
xlabel('horizontal standard deviation');
ylabel('vertical standard deviation'); 
title('color indicates the local density of specs');

%% show asymmetry of receptive fields by peak lighting location on monitor
means_orig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/check_lightingmeans_07_18_2022.mat').means';
means = means_orig(keep,:);
ratios = stds(:,1) ./ stds(:,2);
lb = .5;
ub = 2;
inrange = ratios > lb & ratios < ub;
ratiosp = ratios(inrange);
meansp = means(inrange,:);
figure;
scatter(meansp(:,1), meansp(:,2), [], ratiosp);
colormap jet;
xlabel('horizontal position on monitor (by frame number from sweep)');
ylabel('vertical position on monitor (by frame number from sweep)');
title('asymmetry of receptive field for each spec at its peak lighting position');
a = colorbar;
a.Label.String = 'ratio of x and y standard deviations';

%% show something else that i was interested in/excited about but now can't think of..... hmmm....
