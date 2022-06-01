%% draw the centroids on top of the glitter specs max image
colormap(gray);
imagesc(m);
for cx = 15000:17000%size(C,1)
    axis on;
    hold on;
    plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end