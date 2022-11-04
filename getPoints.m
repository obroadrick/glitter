function pts = getPoints(expdir)
    % assumes that the points are located at the passed experiment's
    % directory at 16pts.mat
    allPts = matfile([expdir '16pts.mat']).arr;
    pin = allPts(1,:);
    pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
    piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
    pin = double([pinx' piny']);
    pts = pin;
end