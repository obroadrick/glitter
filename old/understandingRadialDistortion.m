% need to (for my own understanding) visualize how radial distortion
% coefficients can be fitted to ...

%Pts = [1 1; 2 1; 1 -.8; 1.3 -1.4];
Pts = [];
%Pts(:,1) = randi(100,50,1)+950;
%Pts(:,2) = -50.+randi(100,50,1);
Pts(:,1) = [100; 105; 106; 107; 106; 109; 107; 104; 105; 102; 101];
Pts(:,2) = [100; 75; 65; 40; 11; -2; 23; -40; -50; -78; -80];
%text(Pts(:,1), Pts(:,2), labels, 'FontSize', 15, 'Color','blue');

x0 = [0;0];
f = @(x) errDistortion(Pts, x(1), x(2));
options = optimset('PlotFcns',@optimplotfval);
kfinal = fminsearch(f, x0, options);

k1 = kfinal(1);
k2 = kfinal(2);

for ix=1:size(Pts,1)
    r2 = sum(Pts(ix,:).^2);
    PtsC(ix,:) = Pts(ix,:) .* (1 + k1*r2 + k2*r2^2);
end
figure;
labels = int2str([1:size(Pts,1)]');
plot(Pts(:,1), Pts(:,2), 'b+', 'MarkerSize', 15, 'LineWidth',1);
hold on;
plot(PtsC(:,1), PtsC(:,2), 'rx', 'MarkerSize', 15, 'LineWidth',1);
ylim([-120,120]);
xlim([80,120]);
%text(PtsC(:,1), PtsC(:,2), labels, 'FontSize', 15, 'Color','red');
function error = errDistortion(Pts, k1, k2)
    % error functions says that these points should be as close to a line
    % as possible and so it transforms them for the given k1 k2 and then
    % fits a line and returns a measure of the goodness of fit of that line
    % realistically we would have a line that we know the points are on 
    % or at least we would know that the outer most points form the correct
    % line
    disp(k1);disp(k2);
    for ix=1:size(Pts,1)
        r2 = Pts(ix,1)^2 + Pts(ix,2)^2;
        PtsC(ix,:) = Pts(ix,:) .* (1 + k1*r2 + k2*r2^2);
    end
    Ply = polyfit(PtsC(:,2),PtsC(:,1),1);
    yerr = sum((PtsC(:,1) - (Ply(1).*PtsC(:,2)+Ply(2))).^2);
    error = yerr;
end