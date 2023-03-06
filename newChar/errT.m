% error function for estimating camera position
% given spec positions and their reflected light rays

function error = err(c, S, R)
    % c is candidate camera point
    % S is spec positions
    % R is reflected vectors same order as S

    % for now, just sum the euclidean distances from c to each
    % each of the reflected rays
    dists = [];
    for ix=1:size(S,1)
        a = S(ix,:)';% point on the line
        n = R(ix,:)' ./ norm(R(ix,:)');% unit vector in direction of line
        p = c;% point whose distance is being computed
        dists(ix) = norm((p-a)-(dot((p-a),n,1)*n));
    end
    dists = sort(dists,'descend')';
    %error = sum(dists(1:25));
    weights = logspace(0,3,size(dists,1));
    error = weights * dists;
    % now take those distances and count only the closest half in the sum
    %avgDist = sum(dists) / size(dists,2);
    %error = sum(dists(dists<avgDist));
    %errorFull = sum(dists);
    %disp('new');
    disp(error); 
    %disp(errorFull);
end