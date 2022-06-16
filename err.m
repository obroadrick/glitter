% error function for estimating camera position
% given spec positions and their reflected light rays

function error = err(c, S, R)
    % c is candidate camera point
    % S is spec positions
    % R is reflected vectors same order as S

    % for now, just sum the euclidean distances from c to each
    % each of the reflected rays
    error = 0;
    for ix=1:size(S,1)
        a = S(ix)';% point on the line
        n = R(ix)'/norm(R(ix)');% unit vector in direction of line
        p = c;% point whose distance is being computed
        dist = norm((p-a)-(dot((p-a),n,1)*n));
        error = error + dist;
    end
end