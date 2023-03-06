function pin = loadPoints(path, sixteen)
    % given a path to the 16pts file, load the points into a userful data
    % structure
    allPts = matfile(path).arr;
    pin = [allPts(1,:) allPts(2,:) allPts(3,:) allPts(4,:)];
    for i=1:16
        pinx(i) = pin{i}(1);
        piny(i) = pin{i}(2);
    end
    pin = double([pinx' piny']);
    if ~(sixteen)
        pin = pin(1:4,:);
    end
end