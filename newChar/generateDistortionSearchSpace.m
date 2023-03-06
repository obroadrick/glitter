function kspace = generateDistortionSearchSpace(wiggleRoom, stepSize)
    %% generate the grid search space and then show it
    % we will let both span from -.5 to .5 and then subject them to the
    % constraint |2k1 + 4k2| <= a = .25 (.25 for an example)
    % wiggleRoom is the proportion of its radius that any point can 'move' in or out
    a = wiggleRoom;
    %stepsize = .05;
    overall_lb = -.5;
    overall_ub = .5;
    k2s = linspace(overall_lb,overall_ub,(overall_ub-overall_lb)/stepSize);
    kspace = [];
    numPairs = 0;
    for k2ix=1:size(k2s,2)
        k2 = k2s(k2ix);
        curlb = (-a-4*k2)/2;
        curub = (a-4*k2)/2;
        lb = max(overall_lb, curlb);
        ub = min(overall_ub, curub);
        if lb >= ub
            continue
        end
        curk1s = linspace(lb, ub, ceil((ub-lb)/stepSize));
        for k1ix=1:size(curk1s,2)
            numPairs = numPairs + 1;
            kspace(numPairs,:) = [curk1s(k1ix) k2];
        end
    end
    
    %% plot/show kspace
    plot(kspace(:,1), kspace(:,2), 'r+', 'MarkerSize', 12);
    xlabel('k1');
    ylabel('k2');
    title('distortion coefficients grid search space');
end