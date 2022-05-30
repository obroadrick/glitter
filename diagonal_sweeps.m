%% make images with bars at varying angles for glitter characterization
long = ceil(sqrt(2160^2 + 3840^2));

im = zeros(2160, 3840);

im(712,:) = 1;

colormap(gray);
imagesc(im);

F = fspecial('Gaussian',[15 15],5);

M = imfilter(im, F);
imagesc(M);

%%
Mp = imrotate(M, 30);
imagesc(Mp);

long = ceil(sqrt(2160^2 + 3840^2));
disp(long);