function [idx] = cleanPoints(C, d, buf)
    idx = zeros(1,size(C,1));
    % throw out points that are not within the middle 1-2buf interval of
    % a square
    for ix = 1:size(C,1)
        loc = C(ix,1:2);
        p = (loc(1) / d) - floor(loc(1) / d);
        q = (loc(2) / d) - floor(loc(2) / d);
        if p > buf && p < 1-buf && q > buf && q < 1-buf
            idx(ix) = 1;
        end
        num = floor(loc(1) / d) + floor(loc(2) / d);
        % but then if this is not even one of the black squares, it is set
        % to zero regardless
        if mod(num,2) == 1
            idx(ix) = 0;
        end
        % finally also check if this is just off the board
        x = floor(loc(1) / d);
        y = floor(loc(2) / d);
        if x < -1 || x > 12 || y < -1 || y > 8
            idx(ix) = 0;
        end
    end
    idx = logical(idx);
end
